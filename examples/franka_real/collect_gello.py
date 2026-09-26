#!/usr/bin/env python3
"""Collect one guarded Franka Panda demonstration with GELLO.

The Franka/GELLO/camera services must already be running. Motion is disabled by
default. Real motion requires both ``--enable-motion`` and a Linux evdev
deadman device; releasing the deadman immediately stops the bridge policy.
"""

from __future__ import annotations

import argparse
import contextlib
import datetime as dt
import hashlib
import ipaddress
import json
from pathlib import Path
from pathlib import PurePosixPath
import pickle
import queue
import shutil
import signal
import threading
import time

from franka_runtime import PandaSafetyFilter
from franka_runtime import SafetyConfig
from franka_runtime import TwoStageShutdown
import numpy as np
import zmq

JOINT_LOW = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
JOINT_HIGH = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
CONTROL_HZ = 20.0


def _validate_stable_device_path(value: str, *, directory: str, flag: str) -> None:
    """Require a direct child of a stable Linux by-id directory."""

    path = PurePosixPath(value)
    expected_parent = PurePosixPath(directory)
    if path.parent != expected_parent or not path.name:
        raise ValueError(f"{flag} must name a direct child of {directory}/")


class Deadman:
    def __init__(self, device_path: str, key_code: str):
        import evdev

        self._evdev = evdev
        self._device = evdev.InputDevice(device_path)
        self._device.set_blocking(False)
        try:
            self._key_code = int(key_code)
        except ValueError:
            self._key_code = int(getattr(evdev.ecodes, key_code))
        self.enabled = self._key_code in self._device.active_keys()

    def poll(self) -> bool:
        try:
            # Drain queued events, but never derive the safety state from the
            # incremental stream: SYN_DROPPED can otherwise lose a release and
            # leave the switch stuck true in user space.
            for _event in self._device.read():
                pass
        except BlockingIOError:
            pass
        except OSError:
            self.enabled = False
            return False
        try:
            self.enabled = self._key_code in self._device.active_keys()
        except OSError:
            self.enabled = False
        return self.enabled


class EpisodeWriter:
    def __init__(self, directory: Path, max_pending: int = 256):
        self.directory = directory
        self.directory.mkdir(parents=True, exist_ok=False)
        self._queue: queue.Queue[tuple[dt.datetime, dict] | None] = queue.Queue(maxsize=max_pending)
        self._error: BaseException | None = None
        self._thread = threading.Thread(target=self._run, name="episode-writer", daemon=True)
        self._thread.start()

    def append(self, timestamp: dt.datetime, sample: dict) -> None:
        if self._error is not None:
            raise RuntimeError("episode writer failed") from self._error
        self._queue.put((timestamp, sample), timeout=1.0)

    def close(self) -> None:
        if self._thread.is_alive():
            try:
                self._queue.put(None, timeout=2.0)
            except queue.Full as exc:
                raise RuntimeError("episode writer queue did not drain") from exc
            self._thread.join(timeout=10.0)
        if self._thread.is_alive():
            raise RuntimeError("episode writer did not stop within 10 seconds")
        if self._error is not None:
            raise RuntimeError("episode writer failed") from self._error

    def _run(self) -> None:
        try:
            while True:
                item = self._queue.get()
                if item is None:
                    return
                timestamp, sample = item
                filename = timestamp.astimezone(dt.timezone.utc).strftime(  # noqa: UP017 -- Python 3.8 control PC.
                    "%Y-%m-%dT%H-%M-%S.%fZ.pkl"
                )
                path = self.directory / filename
                with path.open("wb") as stream:
                    pickle.dump(sample, stream, protocol=pickle.HIGHEST_PROTOCOL)
        except BaseException as exc:
            self._error = exc


def _is_loopback(host: str) -> bool:
    if host.lower() == "localhost":
        return True
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False


class _ZmqRequestClient:
    """Timeout-bounded client for GELLO's trusted pickle RPC protocol."""

    def __init__(self, *, host: str, port: int, timeout_ms: int) -> None:
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.REQ)
        self._socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        self._socket.setsockopt(zmq.SNDTIMEO, timeout_ms)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.connect(f"tcp://{host}:{port}")
        self._closed = False

    def exchange(self, request: object) -> object:
        if self._closed:
            raise RuntimeError("pickle RPC client is closed")
        try:
            self._socket.send(pickle.dumps(request, protocol=pickle.HIGHEST_PROTOCOL))
            response = pickle.loads(self._socket.recv())
        except zmq.Again as exc:
            self.close()
            raise RuntimeError("pickle RPC timed out; collection aborted") from exc
        if isinstance(response, dict) and "error" in response:
            raise RuntimeError(f"remote robot rejected the request: {response['error']}")
        return response

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._socket.close()
        self._context.term()


class SafeZmqRobotClient:
    def __init__(self, *, host: str, port: int, timeout_ms: int) -> None:
        self._client = _ZmqRequestClient(host=host, port=port, timeout_ms=timeout_ms)

    def num_dofs(self) -> int:
        result = self._client.exchange({"method": "num_dofs"})
        if isinstance(result, bool) or not isinstance(result, int) or result != 8:
            raise RuntimeError(f"robot bridge must report 8 DoF, got {result!r}")
        return result

    def get_observations(self) -> dict:
        result = self._client.exchange({"method": "get_observations"})
        if not isinstance(result, dict):
            raise RuntimeError(f"robot bridge returned invalid observations: {type(result).__name__}")
        return result

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        self._client.exchange({"method": "command_joint_state", "args": {"joint_state": joint_state}})

    def stop(self, reason: str) -> None:
        self._client.exchange({"method": "stop", "args": {"reason": reason}})

    def close(self) -> None:
        self._client.close()


class SafeZmqCameraClient:
    def __init__(self, *, role: str, host: str, port: int, timeout_ms: int) -> None:
        self._role = role
        self._client = _ZmqRequestClient(host=host, port=port, timeout_ms=timeout_ms)

    def read(self) -> tuple[np.ndarray, object]:
        result = self._client.exchange(None)
        if not isinstance(result, (tuple, list)) or len(result) != 2:  # noqa: UP038 -- Python 3.8 client.
            raise RuntimeError(f"{self._role} camera returned an invalid response")
        image = np.asarray(result[0])
        if image.ndim != 3 or image.shape[-1] != 3 or image.dtype != np.uint8:
            raise RuntimeError(f"{self._role} camera must return uint8 HWC RGB, got {image.shape} {image.dtype}")
        return np.ascontiguousarray(image), result[1]

    def close(self) -> None:
        self._client.close()


def _load_agent(calibration_path: Path, serial_port: str):
    calibration_bytes = calibration_path.read_bytes()
    config = json.loads(calibration_bytes.decode("utf-8"))
    if not isinstance(config, dict):
        raise ValueError("calibration file must contain a JSON object")
    if config.get("calibrated") is not True:
        raise ValueError("calibration file must set calibrated=true after measured offsets are recorded")
    required = (
        "robot_model",
        "gello_device_basename",
        "joint_ids",
        "joint_offsets",
        "joint_signs",
        "gripper_config",
        "start_joints",
        "calibrated_at",
    )
    missing = [key for key in required if key not in config]
    if missing:
        raise ValueError(f"calibration is missing keys: {missing}")
    if config["robot_model"] != "franka_panda":
        raise ValueError("calibration robot_model must be 'franka_panda'")
    gello_device_value = config["gello_device_basename"]
    if not isinstance(gello_device_value, str):
        raise ValueError("calibration gello_device_basename must be a string")
    gello_device_basename = gello_device_value.strip()
    placeholders = ("REQUIRED", "PLACEHOLDER", "CHANGEME", "<", ">")
    if not gello_device_basename or any(token in gello_device_basename.upper() for token in placeholders):
        raise ValueError("calibration gello_device_basename is empty or still a template placeholder")
    if gello_device_basename != Path(serial_port).name:
        raise ValueError("calibration gello_device_basename does not exactly match --gello-port")

    calibrated_at_value = config["calibrated_at"]
    if not isinstance(calibrated_at_value, str):
        raise ValueError("calibrated_at must be an ISO-8601 string with timezone")
    calibrated_at_raw = calibrated_at_value.strip()
    try:
        # Python 3.8 rejects the otherwise valid ISO-8601 trailing ``Z``.
        calibrated_at = dt.datetime.fromisoformat(calibrated_at_raw.replace("Z", "+00:00"))
    except ValueError as exc:
        raise ValueError("calibrated_at must be an ISO-8601 timestamp with timezone") from exc
    if calibrated_at.tzinfo is None:
        raise ValueError("calibrated_at must include a timezone")

    joint_ids = config["joint_ids"]
    if (
        not isinstance(joint_ids, list)
        or len(joint_ids) != 7
        or any(isinstance(value, bool) or not isinstance(value, int) or value <= 0 for value in joint_ids)
        or len(set(joint_ids)) != 7
    ):
        raise ValueError("joint_ids must contain seven unique positive integers")
    joint_offsets = np.asarray(config["joint_offsets"], dtype=np.float64)
    if joint_offsets.shape != (7,) or not np.isfinite(joint_offsets).all():
        raise ValueError("joint_offsets must contain seven finite values")
    joint_signs = np.asarray(config["joint_signs"])
    if joint_signs.shape != (7,) or not np.isin(joint_signs, (-1, 1)).all():
        raise ValueError("joint_signs must contain exactly seven values in {-1, 1}")
    gripper = config["gripper_config"]
    if not isinstance(gripper, list) or len(gripper) != 3:
        raise ValueError("gripper_config must be [unique_id, open_degrees, closed_degrees]")
    gripper_id, gripper_open, gripper_closed = gripper
    if isinstance(gripper_id, bool) or not isinstance(gripper_id, int) or gripper_id <= 0 or gripper_id in joint_ids:
        raise ValueError("gripper_config id must be a unique positive integer")
    endpoints = np.asarray([gripper_open, gripper_closed], dtype=np.float64)
    if not np.isfinite(endpoints).all() or gripper_open == gripper_closed:
        raise ValueError("gripper open/closed endpoints must be distinct finite values")
    start_joints = np.asarray(config["start_joints"], dtype=np.float64)
    if start_joints.shape != (8,) or not np.isfinite(start_joints).all():
        raise ValueError("start_joints must contain eight finite q7+gripper values")
    if np.any(start_joints[:7] < JOINT_LOW) or np.any(start_joints[:7] > JOINT_HIGH):
        raise ValueError("start_joints contains a Panda joint outside physical limits")
    if not 0.0 <= start_joints[7] <= 1.0:
        raise ValueError("start_joints gripper_closed_fraction must be in [0, 1]")

    from gello.agents.gello_agent import DynamixelRobotConfig
    from gello.agents.gello_agent import GelloAgent

    hardware = DynamixelRobotConfig(
        joint_ids=tuple(joint_ids),
        joint_offsets=tuple(joint_offsets.tolist()),
        joint_signs=tuple(int(value) for value in joint_signs),
        gripper_config=(gripper_id, float(gripper_open), float(gripper_closed)),
    )
    return (
        GelloAgent(
            port=serial_port,
            dynamixel_config=hardware,
            start_joints=start_joints,
        ),
        hashlib.sha256(calibration_bytes).hexdigest(),
    )


def _current_command(obs: dict) -> np.ndarray:
    joint_positions = np.asarray(obs["joint_positions"], dtype=np.float64).reshape(-1)
    if joint_positions.size < 7 or not np.isfinite(joint_positions[:7]).all():
        raise ValueError(f"robot returned invalid joint_positions: {joint_positions.shape}")
    if np.any(joint_positions[:7] < JOINT_LOW) or np.any(joint_positions[:7] > JOINT_HIGH):
        raise ValueError("measured Panda joint position is outside limits")
    gripper_closed = float(np.asarray(obs["gripper_position"]).reshape(-1)[0])
    if not np.isfinite(gripper_closed) or not 0.0 <= gripper_closed <= 1.0:
        raise ValueError("measured gripper_closed_fraction must be finite and in [0, 1]")
    return np.concatenate([joint_positions[:7], [gripper_closed]])


def _safe_command(
    target: np.ndarray,
    obs: dict,
    safety_filter: PandaSafetyFilter,
    *,
    dt_s: float,
) -> np.ndarray:
    target = np.asarray(target, dtype=np.float64).reshape(-1)
    if target.shape != (8,) or not np.isfinite(target).all():
        raise ValueError(f"GELLO target must be 8 finite values, got {target.shape}")
    current = _current_command(obs)
    return np.asarray(safety_filter.filter(target, current, dt_s=dt_s), dtype=np.float64)


def _stop_after_deadman_release(robot: SafeZmqRobotClient) -> None:
    """Stop immediately; never wait for another motion RPC before termination."""
    robot.stop("deadman released")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--gello-port", required=True, help="Use /dev/serial/by-id/... only")
    parser.add_argument("--robot-host", default="127.0.0.1")
    parser.add_argument("--robot-port", type=int, default=6001)
    parser.add_argument("--camera-host", default="127.0.0.1")
    parser.add_argument("--exterior-port", type=int, default=5000)
    parser.add_argument("--wrist-port", type=int, default=5001)
    parser.add_argument("--rpc-timeout-ms", type=int, default=1000)
    parser.add_argument("--allow-remote-robot-pickle", action="store_true")
    parser.add_argument("--allow-remote-camera-pickle", action="store_true")
    parser.add_argument("--data-dir", type=Path, default=Path("~/bc_data/franka_gello"))
    parser.add_argument("--duration-s", type=float, default=60.0)
    parser.add_argument("--control-hz", type=float, default=CONTROL_HZ, help="Fixed by the policy contract at 20 Hz")
    parser.add_argument("--max-start-delta-rad", type=float, default=0.25)
    parser.add_argument("--max-joint-step-rad", type=float, default=0.025)
    parser.add_argument("--max-joint-acceleration-rad-s2", type=float, default=3.0)
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--deadman-device", help="Linux evdev path, for example /dev/input/by-id/...")
    parser.add_argument("--deadman-code", default="KEY_SPACE")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not args.task.strip():
        raise ValueError("task must not be empty")
    _validate_stable_device_path(
        args.gello_port,
        directory="/dev/serial/by-id",
        flag="--gello-port",
    )
    if args.enable_motion and not args.deadman_device:
        raise ValueError("real motion requires --deadman-device")
    if args.deadman_device:
        _validate_stable_device_path(
            args.deadman_device,
            directory="/dev/input/by-id",
            flag="--deadman-device",
        )
    if not np.isclose(args.control_hz, CONTROL_HZ):
        raise ValueError(f"--control-hz is fixed by the policy contract at {CONTROL_HZ:g}")
    if args.max_joint_step_rad <= 0 or args.max_joint_acceleration_rad_s2 <= 0 or args.max_start_delta_rad <= 0:
        raise ValueError("joint safety limits must be positive")
    if args.duration_s == 0:
        raise ValueError("--duration-s must be positive, or negative to run until interrupted")
    if args.rpc_timeout_ms <= 0:
        raise ValueError("--rpc-timeout-ms must be positive")
    for name in ("robot_port", "exterior_port", "wrist_port"):
        port = getattr(args, name)
        if not 1 <= port <= 65535:
            raise ValueError(f"--{name.replace('_', '-')} must be in [1, 65535]")
    if args.exterior_port == args.wrist_port:
        raise ValueError("exterior and wrist camera ports must be different")
    if not _is_loopback(args.robot_host) and not args.allow_remote_robot_pickle:
        raise ValueError("remote robot pickle RPC requires --allow-remote-robot-pickle on a trusted private network")
    if not _is_loopback(args.camera_host) and not args.allow_remote_camera_pickle:
        raise ValueError("remote camera pickle RPC requires --allow-remote-camera-pickle on a trusted private network")

    calibration_path = args.calibration.expanduser().resolve()
    if not calibration_path.is_file():
        raise FileNotFoundError(calibration_path)

    robot: SafeZmqRobotClient | None = None
    cameras: dict[str, SafeZmqCameraClient] = {}
    writer: EpisodeWriter | None = None
    in_progress: Path | None = None
    final_path: Path | None = None
    recorded = 0
    outcome = "aborted"
    shutdown_requested = threading.Event()
    shutdown = TwoStageShutdown(shutdown_requested.set)

    try:
        signal.signal(signal.SIGINT, shutdown.handle_signal)
        signal.signal(signal.SIGTERM, shutdown.handle_signal)
        from gello.env import RobotEnv

        agent, calibration_sha256 = _load_agent(calibration_path, args.gello_port)
        robot = SafeZmqRobotClient(host=args.robot_host, port=args.robot_port, timeout_ms=args.rpc_timeout_ms)
        cameras["base"] = SafeZmqCameraClient(
            role="exterior", host=args.camera_host, port=args.exterior_port, timeout_ms=args.rpc_timeout_ms
        )
        cameras["wrist"] = SafeZmqCameraClient(
            role="wrist", host=args.camera_host, port=args.wrist_port, timeout_ms=args.rpc_timeout_ms
        )
        env = RobotEnv(robot, control_rate_hz=args.control_hz, camera_dict=cameras)
        deadman = Deadman(args.deadman_device, args.deadman_code) if args.enable_motion else None
        obs = env.get_obs()
        leader = np.asarray(agent.act(obs), dtype=np.float64)
        follower = _current_command(obs)
        start_delta = np.abs(leader[:7] - follower[:7])
        if np.max(start_delta) > args.max_start_delta_rad:
            raise RuntimeError(f"leader/follower start mismatch: per-joint rad={start_delta.tolist()}")

        data_root = args.data_dir.expanduser().resolve()
        data_root.mkdir(parents=True, exist_ok=True)
        episode_name = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")  # noqa: UP017
        in_progress = data_root / f".{episode_name}.inprogress"
        final_path = data_root / episode_name
        if in_progress.exists() or final_path.exists():
            raise FileExistsError(f"episode path already exists: {episode_name}")
        writer = EpisodeWriter(in_progress)

        print("[ready] alignment passed")
        print("[shadow] motion disabled" if deadman is None else "[armed] hold the deadman to move and record")
        if deadman is not None and not deadman.poll():
            robot.stop("deadman not held at episode start")
            raise RuntimeError("deadman must already be held when the episode starts; bridge stopped")
        safety_filter = PandaSafetyFilter(
            SafetyConfig(
                max_joint_step_rad=args.max_joint_step_rad,
                max_joint_acceleration_rad_s2=args.max_joint_acceleration_rad_s2,
            )
        )
        started = time.monotonic()
        outcome = "complete"
        while args.duration_s <= 0 or time.monotonic() - started < args.duration_s:
            target = _safe_command(agent.act(obs), obs, safety_filter, dt_s=1.0 / args.control_hz)
            enabled = deadman.poll() if deadman is not None else False
            if deadman is not None and not enabled:
                _stop_after_deadman_release(robot)
                outcome = "deadman_released"
                print("[episode-stop] bridge stopped after deadman release")
                break
            applied = target if enabled else _current_command(obs)
            if enabled:
                sample = {
                    key: np.array(value, copy=True) if isinstance(value, np.ndarray) else value
                    for key, value in obs.items()
                }
                sample["control"] = applied.astype(np.float32)
                sample["task"] = args.task.strip()
                sample["schema_id"] = "franka-runtime/v1"
                writer.append(dt.datetime.now(dt.timezone.utc), sample)  # noqa: UP017
                recorded += 1
            obs = env.step(applied) if args.enable_motion else env.get_obs()
            if not args.enable_motion:
                time.sleep(1.0 / args.control_hz)
    except KeyboardInterrupt:
        outcome = "operator_stopped"
    except BaseException:
        outcome = "aborted"
        raise
    finally:
        shutdown.begin_cleanup()
        cleanup_error: BaseException | None = None
        try:
            if args.enable_motion and robot is not None:
                try:
                    robot.stop(f"collector exit: {outcome}")
                except BaseException as exc:
                    outcome = "aborted"
                    cleanup_error = RuntimeError(
                        "HARD STOP FAILURE: robot bridge did not confirm stop; use the external E-stop"
                    )
                    cleanup_error.__cause__ = exc
            if writer is not None and in_progress is not None and final_path is not None:
                try:
                    writer.close()
                    metadata = {
                        "schema_id": "franka-runtime/v1",
                        "task": args.task.strip(),
                        "outcome": outcome,
                        "recorded_frames": recorded,
                        "control_hz": args.control_hz,
                        "max_joint_step_rad": args.max_joint_step_rad,
                        "max_joint_acceleration_rad_s2": args.max_joint_acceleration_rad_s2,
                        "camera_ports": {"exterior": args.exterior_port, "wrist": args.wrist_port},
                        "color_space": "rgb_uint8_hwc",
                        "gripper_semantics": "closed_fraction_0_open_1_closed",
                        "gello_device_sha256": hashlib.sha256(Path(args.gello_port).name.encode()).hexdigest(),
                        "calibration_sha256": calibration_sha256,
                    }
                    (in_progress / "episode.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
                    if outcome == "aborted" or recorded < 2:
                        suffix = ".inprogress"
                        if not in_progress.name.endswith(suffix):
                            raise RuntimeError(f"unexpected in-progress episode name: {in_progress.name}")
                        discarded = in_progress.with_name(in_progress.name[: -len(suffix)] + ".discarded")
                        shutil.move(str(in_progress), str(discarded))
                        print(f"[discarded] {discarded} frames={recorded}")
                    else:
                        in_progress.rename(final_path)
                        print(f"[saved] {final_path} frames={recorded} outcome={outcome}")
                except BaseException as exc:
                    if cleanup_error is None:
                        cleanup_error = exc
        finally:
            if robot is not None:
                robot.close()
            for camera in cameras.values():
                with contextlib.suppress(BaseException):
                    camera.close()
        if cleanup_error is not None:
            raise RuntimeError("collector cleanup failed; episode was not approved") from cleanup_error


if __name__ == "__main__":
    main()
