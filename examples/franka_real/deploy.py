#!/usr/bin/env python3
"""Run a fail-closed OpenPI policy against a Polymetis Panda.

The default mode is read-only shadow evaluation. Real motion is possible only
with ``--enable-motion`` followed by typing ``ARM`` at the terminal. The script
never homes the robot and never performs automatic error recovery.

Camera frames are treated as RGB exactly as received. Use
``launch_cameras.py`` to bind the physical camera serial numbers to the two
explicit role ports; this client never guesses roles or swaps color channels.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable, Mapping
import contextlib
from dataclasses import dataclass
import ipaddress
import pickle
import queue
import signal
import threading
import time
from typing import Any, Protocol

from franka_runtime import EXPECTED_POLICY_METADATA
from franka_runtime import PANDA_JOINT_LIMIT_HIGH
from franka_runtime import PANDA_JOINT_LIMIT_LOW
from franka_runtime import PANDA_JOINT_LIMIT_MARGIN_RAD
from franka_runtime import ActionChunkQueue
from franka_runtime import ChunkQueueConfig
from franka_runtime import PandaSafetyFilter
from franka_runtime import PolicyMetadata
from franka_runtime import SafetyConfig
from franka_runtime import TwoStageShutdown
from franka_runtime import join_observation
from franka_runtime import validate_action_chunk
from franka_runtime import validate_observation
from franka_runtime import validate_policy_metadata
import numpy as np

MAX_GRIPPER_WIDTH_M = 0.08


class PolicyLike(Protocol):
    def infer(self, obs: dict[str, Any]) -> Mapping[str, Any]: ...

    def get_server_metadata(self) -> Mapping[str, Any]: ...

    def close(self) -> None: ...


def _as_numpy(value: object, *, shape: tuple[int, ...], name: str) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    result = np.asarray(value, dtype=np.float64)
    if result.shape != shape or not np.isfinite(result).all():
        raise RuntimeError(f"{name} must be finite with shape {shape}, got {result.shape}")
    return result


def _validate_rgb_frame(value: object, *, role: str) -> np.ndarray:
    """Validate an RGB frame without changing its channel order."""

    frame = np.asarray(value)
    if frame.ndim != 3 or frame.shape[-1] != 3 or min(frame.shape[:2]) < 1:
        raise RuntimeError(f"{role} camera must return an HWC RGB image, got {frame.shape}")
    if frame.dtype != np.uint8:
        raise RuntimeError(f"{role} camera must return uint8 RGB, got {frame.dtype}")
    return np.ascontiguousarray(frame)


def _build_policy_observation(*, exterior_rgb: object, wrist_rgb: object, state: object, prompt: str) -> dict[str, Any]:
    if not prompt.strip():
        raise ValueError("prompt must not be empty")
    return {
        "exterior_image": _validate_rgb_frame(exterior_rgb, role="exterior"),
        "wrist_image": _validate_rgb_frame(wrist_rgb, role="wrist"),
        "state": validate_observation(state),
        "prompt": prompt.strip(),
    }


def _require_expected_metadata(value: Mapping[str, Any]) -> PolicyMetadata:
    metadata = validate_policy_metadata(value)
    if metadata != EXPECTED_POLICY_METADATA:
        raise RuntimeError(
            "policy metadata does not exactly match the deployed Franka contract: "
            f"expected={EXPECTED_POLICY_METADATA.to_mapping()} actual={metadata.to_mapping()}"
        )
    return metadata


def _request_action_chunk(
    policy: PolicyLike,
    observation: dict[str, Any],
    *,
    expected_horizon: int,
) -> np.ndarray:
    result = policy.infer(observation)
    if not isinstance(result, Mapping) or "actions" not in result:
        raise RuntimeError("policy response must be a mapping containing 'actions'")
    return validate_action_chunk(result["actions"], expected_horizon=expected_horizon)


def _is_loopback(host: str) -> bool:
    if host.lower() == "localhost":
        return True
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False


class RGBZmqCamera:
    """Small timeout-bounded client for GELLO's trusted pickle camera RPC."""

    def __init__(self, *, role: str, host: str, port: int, timeout_ms: int) -> None:
        import zmq

        self._role = role
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.REQ)
        self._socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        self._socket.setsockopt(zmq.SNDTIMEO, timeout_ms)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.connect(f"tcp://{host}:{port}")

    def read(self) -> np.ndarray:
        self._socket.send(pickle.dumps(None, protocol=pickle.HIGHEST_PROTOCOL))
        # Pickle is why non-loopback camera endpoints require an explicit
        # trusted-network opt-in in _validate_args.
        response = pickle.loads(self._socket.recv())
        if not isinstance(response, (tuple, list)) or len(response) != 2:  # noqa: UP038 -- Python 3.8 client.
            raise RuntimeError(f"{self._role} camera returned an invalid response")
        rgb, _depth = response
        return _validate_rgb_frame(rgb, role=self._role)

    def close(self) -> None:
        self._socket.close()
        self._context.term()


class _LatestStateReader:
    """Continuously read state so a wedged gRPC read cannot block safety logic."""

    def __init__(self, read_once: Callable[[], np.ndarray], *, poll_hz: float) -> None:
        self._read_once = read_once
        self._period_s = 1.0 / poll_hz
        self._condition = threading.Condition()
        self._state: np.ndarray | None = None
        self._captured_at_s = 0.0
        self._error: BaseException | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="robot-state", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        try:
            while not self._stop.is_set():
                started_s = time.monotonic()
                state = validate_observation(self._read_once())
                captured_at_s = time.monotonic()
                with self._condition:
                    self._state = state
                    self._captured_at_s = captured_at_s
                    self._condition.notify_all()
                self._stop.wait(max(0.0, self._period_s - (captured_at_s - started_s)))
        except BaseException as exc:
            with self._condition:
                self._error = exc
                self._condition.notify_all()
            self._stop.set()

    def read(self, *, timeout_s: float) -> np.ndarray:
        deadline_s = time.monotonic() + timeout_s
        with self._condition:
            while self._state is None and self._error is None and not self._stop.is_set():
                remaining_s = deadline_s - time.monotonic()
                if remaining_s <= 0:
                    break
                self._condition.wait(remaining_s)
            if self._error is not None:
                raise RuntimeError("Polymetis state reader failed") from self._error
            if self._state is None:
                raise TimeoutError("timed out waiting for the first Polymetis state")
            state = self._state.copy()
            captured_at_s = self._captured_at_s
        age_s = time.monotonic() - captured_at_s
        if age_s >= timeout_s:
            raise TimeoutError(f"Polymetis state is stale ({age_s:.3f}s)")
        return state

    def close(self, *, join_timeout_s: float) -> None:
        self._stop.set()
        with self._condition:
            self._condition.notify_all()
        self._thread.join(timeout=join_timeout_s)
        if self._thread.is_alive():
            raise RuntimeError("state RPC remains in flight during shutdown")


class _LatestArmWorker:
    """Latest-wins worker for potentially blocking arm update RPCs."""

    def __init__(self, robot: Any, on_late_completion: Callable[[str], None]) -> None:
        self._robot = robot
        self._on_late_completion = on_late_completion
        self._commands: queue.Queue[tuple[np.ndarray, int]] = queue.Queue(maxsize=1)
        self._errors: queue.Queue[BaseException] = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._generation = 0
        self._last_completed_s = 0.0
        # Keep the process alive until a late arm RPC can issue its compensating
        # terminate. A second OS signal is the explicit escape for a permanent
        # transport hang while the external E-stop is held.
        self._thread = threading.Thread(target=self._run, name="arm-command", daemon=False)
        self._thread.start()

    def reset_freshness(self, *, now_s: float) -> None:
        with self._lock:
            self._last_completed_s = now_s

    def submit(self, target: np.ndarray) -> None:
        command = np.asarray(target, dtype=np.float64).copy()
        with self._lock:
            if self._stop.is_set():
                raise RuntimeError("arm command worker is stopped")
            item = (command, self._generation)
            try:
                self._commands.put_nowait(item)
            except queue.Full:
                with contextlib.suppress(queue.Empty):
                    self._commands.get_nowait()
                self._commands.put_nowait(item)

    def raise_if_failed_or_stale(self, *, now_s: float, timeout_s: float) -> None:
        try:
            error = self._errors.get_nowait()
        except queue.Empty:
            error = None
        if error is not None:
            raise RuntimeError("arm command failed") from error
        with self._lock:
            last_completed_s = self._last_completed_s
        if last_completed_s > 0.0 and now_s - last_completed_s >= timeout_s:
            raise TimeoutError(f"arm command acknowledgement is stale ({now_s - last_completed_s:.3f}s)")

    def _run(self) -> None:
        import torch

        while not self._stop.is_set():
            try:
                target, generation = self._commands.get(timeout=0.05)
            except queue.Empty:
                continue
            with self._lock:
                if self._stop.is_set() or generation != self._generation:
                    return
            request_started = False
            late_completion = False
            try:
                # stop() may race immediately after this check. If so, the
                # post-RPC generation check below performs a second terminate
                # after the late update returns.
                request_started = True
                self._robot.update_desired_joint_positions(torch.as_tensor(target, dtype=torch.float32))
                with self._lock:
                    late_completion = self._stop.is_set() or generation != self._generation
                    if not late_completion:
                        self._last_completed_s = time.monotonic()
                if late_completion:
                    raise RuntimeError("arm update completed after stop")
            except BaseException as exc:
                with contextlib.suppress(queue.Full):
                    self._errors.put_nowait(exc)
                with self._lock:
                    late_completion = request_started and (self._stop.is_set() or generation != self._generation)
                    self._stop.set()
                    self._generation += 1
            finally:
                if late_completion:
                    self._on_late_completion("arm update completed after stop")

    def stop(self) -> None:
        with self._lock:
            if not self._stop.is_set():
                self._generation += 1
            self._stop.set()
            with contextlib.suppress(queue.Empty):
                while True:
                    self._commands.get_nowait()

    def close(self, *, join_timeout_s: float = 0.1) -> None:
        self.stop()
        self._thread.join(timeout=join_timeout_s)
        if self._thread.is_alive():
            raise RuntimeError("arm update RPC remains in flight; use the external E-stop")


class _LatestGripperWorker:
    """Latest-wins gripper worker with an independently bounded stop RPC."""

    def __init__(self, gripper: Any) -> None:
        self._gripper = gripper
        self._commands: queue.Queue[tuple[float, float, float, int]] = queue.Queue(maxsize=1)
        self._errors: queue.Queue[BaseException] = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._generation = 0
        self._stop_threads: list[threading.Thread] = []
        self._stop_events: list[threading.Event] = []
        self._stop_errors: list[BaseException | None] = []
        self._stop_started_s: list[float] = []
        self._stop_finished_s: list[float | None] = []
        self._ever_submitted = False
        self._goto_complete = threading.Event()
        self._goto_complete.set()
        # Both the command worker and the lazily-created stop guardian are
        # non-daemon. A second OS signal remains the explicit escape for a
        # permanent transport hang after the external E-stop is applied.
        self._thread = threading.Thread(target=self._run, name="gripper-command", daemon=False)
        self._thread.start()

    def submit(self, *, width: float, speed: float, force: float) -> None:
        with self._lock:
            if self._stop.is_set():
                raise RuntimeError("gripper command worker is stopped")
            self._ever_submitted = True
            command = (width, speed, force, self._generation)
            try:
                self._commands.put_nowait(command)
            except queue.Full:
                with contextlib.suppress(queue.Empty):
                    self._commands.get_nowait()
                self._commands.put_nowait(command)

    def raise_if_failed(self) -> None:
        with self._lock:
            stop_errors = tuple(error for error in self._stop_errors if error is not None)
        if stop_errors:
            raise RuntimeError("gripper stop RPC failed") from stop_errors[0]
        try:
            error = self._errors.get_nowait()
        except queue.Empty:
            return
        raise RuntimeError("gripper command failed") from error

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                width, speed, force, generation = self._commands.get(timeout=0.05)
            except queue.Empty:
                continue
            with self._lock:
                if self._stop.is_set() or generation != self._generation:
                    return
                self._goto_complete.clear()
                request_started = True
            try:
                self._gripper.goto(width=width, speed=speed, force=force)
            except BaseException as exc:
                with contextlib.suppress(queue.Full):
                    self._errors.put_nowait(exc)
                with self._lock:
                    if not self._stop.is_set():
                        self._generation += 1
                    self._stop.set()
                    if not self._stop_threads:
                        self._start_stop_attempt_locked()
            finally:
                with self._lock:
                    late_completion = request_started and (self._stop.is_set() or generation != self._generation)
                    if late_completion:
                        # The first Stop deliberately does not wait for Goto.
                        # A second Stop after a late response closes the
                        # remaining request-order race.
                        self._start_stop_attempt_locked()
                    # Publish completion only after any compensation Stop has
                    # been created, so waiters cannot observe an incomplete
                    # attempt list.
                    self._goto_complete.set()

    def stop(self, *, ensure_hardware_stop: bool = False) -> bool:
        with self._lock:
            if not self._stop.is_set():
                self._generation += 1
            self._stop.set()
            with contextlib.suppress(queue.Empty):
                while True:
                    self._commands.get_nowait()
            should_send_stop = ensure_hardware_stop or self._ever_submitted
            if should_send_stop and not self._stop_threads:
                self._start_stop_attempt_locked()
            return bool(self._stop_threads)

    def _start_stop_attempt_locked(self) -> None:
        attempt = len(self._stop_threads)
        event = threading.Event()
        thread = threading.Thread(
            target=self._send_stop,
            args=(attempt,),
            name=f"gripper-stop-{attempt + 1}",
            daemon=False,
        )
        self._stop_events.append(event)
        self._stop_errors.append(None)
        self._stop_started_s.append(time.monotonic())
        self._stop_finished_s.append(None)
        self._stop_threads.append(thread)
        thread.start()

    def _send_stop(self, attempt: int) -> None:
        error: BaseException | None = None
        try:
            self._gripper.stop()
        except BaseException as exc:
            error = exc
        finally:
            with self._lock:
                self._stop_errors[attempt] = error
                self._stop_finished_s[attempt] = time.monotonic()
            self._stop_events[attempt].set()

    def wait_for_stop(self, *, timeout_s: float) -> None:
        with self._lock:
            started_s = None if not self._stop_threads else self._stop_started_s[0]
        if started_s is None:
            raise RuntimeError("gripper stop RPC was not requested")

        deadline_s = started_s + timeout_s

        def wait_for_attempt(attempt: int) -> None:
            with self._lock:
                event = self._stop_events[attempt]
            remaining_s = max(0.0, deadline_s - time.monotonic())
            if not event.wait(remaining_s):
                raise TimeoutError("gripper stop RPC timed out")
            with self._lock:
                finished_s = self._stop_finished_s[attempt]
            if finished_s is None or finished_s > deadline_s:
                raise TimeoutError("gripper stop RPC timed out")

        # The first Stop must be dispatched immediately, but shutdown is not
        # complete while a pre-stop Goto can still return. Its finally block
        # creates the compensating Stop before publishing Goto completion.
        wait_for_attempt(0)
        if not self._goto_complete.wait(max(0.0, deadline_s - time.monotonic())):
            raise TimeoutError("gripper Goto RPC remains in flight after Stop")

        attempt = 1
        while True:
            with self._lock:
                attempt_count = len(self._stop_threads)
            if attempt >= attempt_count:
                break
            wait_for_attempt(attempt)
            attempt += 1
        self.raise_if_failed()

    def close(self, *, join_timeout_s: float = 0.1) -> None:
        stop_started = self.stop()
        failure: BaseException | None = None
        if stop_started:
            try:
                self.wait_for_stop(timeout_s=join_timeout_s)
            except BaseException as exc:
                failure = exc
        self._thread.join(timeout=join_timeout_s)
        if failure is None and self._thread.is_alive():
            failure = RuntimeError("gripper command worker did not stop; use the external E-stop")
        for stop_thread in self._stop_threads:
            stop_thread.join(timeout=0.0)
            if failure is None and stop_thread.is_alive():
                failure = RuntimeError("gripper stop guardian did not exit; use the external E-stop")
        if failure is not None:
            raise failure


class PolymetisPanda:
    """Read state in either mode; expose motion only after explicit arming."""

    def __init__(self, args: argparse.Namespace) -> None:
        from polymetis import GripperInterface
        from polymetis import RobotInterface

        self._args = args
        self._robot = RobotInterface(ip_address=args.polymetis_host, port=args.arm_port)
        self._gripper = GripperInterface(ip_address=args.polymetis_host, port=args.gripper_port)
        self._state_lock = threading.Lock()
        self._armed = False
        self._arming = False
        self._stopped = False
        self._termination_failure: RuntimeError | None = None
        self._stop_complete = threading.Event()
        self._arming_thread: threading.Thread | None = None
        self._termination_threads: list[threading.Thread] = []
        self._termination_threads_lock = threading.Lock()
        self._arm_worker = _LatestArmWorker(self._robot, self._terminate_policy_bounded)
        self._gripper_worker = _LatestGripperWorker(self._gripper)
        self._last_gripper_target: float | None = None
        self._last_gripper_command_s = 0.0
        self._state_reader = _LatestStateReader(self._read_state_once, poll_hz=args.state_poll_hz)

    @property
    def armed(self) -> bool:
        with self._state_lock:
            return self._armed and not self._stopped

    def _read_state_once(self) -> np.ndarray:
        joints = _as_numpy(self._robot.get_joint_positions(), shape=(7,), name="joint positions")
        if np.any(joints < PANDA_JOINT_LIMIT_LOW) or np.any(joints > PANDA_JOINT_LIMIT_HIGH):
            raise RuntimeError("measured joint position is outside Panda limits")
        width = float(self._gripper.get_state().width)
        if not np.isfinite(width) or not 0.0 <= width <= MAX_GRIPPER_WIDTH_M:
            raise RuntimeError("measured gripper width is invalid or outside [0, 0.08] m")
        closed = 1.0 - width / MAX_GRIPPER_WIDTH_M
        return join_observation(joints, float(closed))

    def read_state(self) -> np.ndarray:
        return self._state_reader.read(timeout_s=self._args.state_timeout_s)

    def _terminate_policy_bounded(self, reason: str) -> None:
        completed = threading.Event()
        errors: list[BaseException] = []

        def terminate() -> None:
            try:
                self._robot.terminate_current_policy()
            except BaseException as exc:
                errors.append(exc)
            finally:
                completed.set()

        terminate_thread = threading.Thread(target=terminate, name="terminate-policy", daemon=False)
        with self._termination_threads_lock:
            self._termination_threads.append(terminate_thread)
        terminate_thread.start()
        failure: RuntimeError | None = None
        if not completed.wait(self._args.stop_timeout_s):
            failure = RuntimeError(
                f"HARD STOP FAILURE: {reason}; terminate_current_policy timed out; use the external E-stop"
            )
        elif errors:
            failure = RuntimeError(
                f"HARD STOP FAILURE: {reason}; terminate_current_policy failed; use the external E-stop"
            )
            failure.__cause__ = errors[0]
        if failure is not None:
            self._record_termination_failure(failure)
            # This log happens only after the bounded software stop attempt;
            # safety does not depend on stdout/stderr being writable.
            print(f"[HARD-STOP-FAILURE] {failure}", flush=True)

    def _record_termination_failure(self, failure: RuntimeError) -> None:
        with self._state_lock:
            if self._termination_failure is None:
                self._termination_failure = failure

    def _wait_for_gripper_stop_bounded(self, reason: str) -> None:
        try:
            self._gripper_worker.wait_for_stop(timeout_s=self._args.stop_timeout_s)
        except BaseException as exc:
            failure = RuntimeError(f"HARD STOP FAILURE: {reason}; {exc}; use the external E-stop")
            failure.__cause__ = exc
            self._record_termination_failure(failure)
            print(f"[HARD-STOP-FAILURE] {failure}", flush=True)

    def arm(self, cancel_event: threading.Event) -> None:
        with self._state_lock:
            if self._stopped:
                raise RuntimeError("a stopped runtime cannot be re-armed")
        confirmation = input(
            "Clear the workspace, verify the external E-stop, hold the activation device, then type ARM: "
        )
        if confirmation.strip() != "ARM":
            raise RuntimeError("arming cancelled; exact confirmation ARM was not entered")
        if cancel_event.is_set():
            raise RuntimeError("arming cancelled by shutdown request")
        with self._state_lock:
            if self._stopped:
                raise RuntimeError("runtime stopped while waiting for ARM confirmation")
            self._arming = True

        completed = threading.Event()
        abandoned = threading.Event()
        errors: list[BaseException] = []

        def start_policy() -> None:
            try:
                self._robot.start_joint_impedance()
            except BaseException as exc:
                errors.append(exc)
            finally:
                completed.set()
                if abandoned.is_set():
                    self._terminate_policy_bounded("late arming response")

        # This guardian is deliberately non-daemon. If the RPC outlives its
        # deadline, process shutdown waits for its late-response termination
        # instead of silently abandoning a possibly active policy.
        start_thread = threading.Thread(target=start_policy, name="start-joint-impedance", daemon=False)
        self._arming_thread = start_thread
        try:
            start_thread.start()
            deadline_s = time.monotonic() + self._args.arm_timeout_s
            while not completed.wait(min(0.05, max(0.0, deadline_s - time.monotonic()))):
                if cancel_event.is_set():
                    raise RuntimeError("arming interrupted by shutdown")
                if time.monotonic() >= deadline_s:
                    raise TimeoutError("start_joint_impedance timed out")
            if errors:
                raise RuntimeError("start_joint_impedance failed") from errors[0]
            with self._state_lock:
                if self._stopped or cancel_event.is_set():
                    raise RuntimeError("runtime stopped while arming")
                self._arming = False
                self._armed = True
        except BaseException:
            # If the request reached the server but its response was lost, an
            # impedance policy may exist even though arming appeared to fail.
            abandoned.set()
            self.stop("arming failed")
            if start_thread.ident is not None and not completed.is_set():
                failure = RuntimeError(
                    "HARD STOP FAILURE: start_joint_impedance remains in flight after arming failed; "
                    "policy state is unknown; use the external E-stop"
                )
                self._record_termination_failure(failure)
            raise
        self._arm_worker.reset_freshness(now_s=time.monotonic())

    def command_arm(self, joint_target: np.ndarray) -> None:
        if not self.armed:
            raise RuntimeError("arm command rejected because the runtime is not armed")
        target = _as_numpy(joint_target, shape=(7,), name="joint target")
        if np.any(target < PANDA_JOINT_LIMIT_LOW + PANDA_JOINT_LIMIT_MARGIN_RAD) or np.any(
            target > PANDA_JOINT_LIMIT_HIGH - PANDA_JOINT_LIMIT_MARGIN_RAD
        ):
            raise RuntimeError("joint command is outside conservative Panda limits")
        self._arm_worker.submit(target)

    def command_gripper(self, closed_fraction: float, *, now_s: float) -> None:
        if not self.armed:
            raise RuntimeError("gripper command rejected because the runtime is not armed")
        target = float(closed_fraction)
        if not np.isfinite(target) or not 0.0 <= target <= 1.0:
            raise RuntimeError(f"invalid gripper closed fraction: {target!r}")
        due = now_s - self._last_gripper_command_s >= self._args.gripper_min_interval_s
        changed = (
            self._last_gripper_target is None or abs(target - self._last_gripper_target) >= self._args.gripper_deadband
        )
        if due and changed:
            self._gripper_worker.submit(
                width=(1.0 - target) * MAX_GRIPPER_WIDTH_M,
                speed=self._args.gripper_speed,
                force=self._args.gripper_force,
            )
            self._last_gripper_target = target
            self._last_gripper_command_s = now_s

    def raise_if_failed(self) -> None:
        with self._state_lock:
            termination_failure = self._termination_failure
        if termination_failure is not None:
            raise termination_failure
        self._gripper_worker.raise_if_failed()
        if self.armed:
            self._arm_worker.raise_if_failed_or_stale(
                now_s=time.monotonic(), timeout_s=self._args.command_watchdog_timeout_s
            )

    def stop(self, reason: str) -> None:
        with self._state_lock:
            first_stop = not self._stopped
            self._stopped = True
            was_active = self._armed or self._arming
            self._armed = False
            self._arming = False
        # Clear queued work and launch the Hand Stop guardian before waiting
        # on either network call. Arm termination therefore proceeds while the
        # independently bounded gripper stop RPC is in flight.
        self._arm_worker.stop()
        gripper_stop_started = self._gripper_worker.stop(ensure_hardware_stop=was_active)
        if first_stop:
            try:
                if was_active:
                    self._terminate_policy_bounded(reason)
                    print(f"[stopped] {reason}; restart and re-arm manually", flush=True)
                if gripper_stop_started:
                    self._wait_for_gripper_stop_bounded(reason)
            finally:
                self._stop_complete.set()
        elif not self._stop_complete.wait(self._args.stop_timeout_s + 0.5):
            self._record_termination_failure(
                RuntimeError(
                    "HARD STOP FAILURE: concurrent stop did not complete; policy state is unknown; "
                    "use the external E-stop"
                )
            )

    def close(self) -> None:
        self.stop("runtime shutdown")
        close_errors: list[BaseException] = []
        for close in (
            lambda: self._arm_worker.close(join_timeout_s=self._args.stop_timeout_s),
            lambda: self._gripper_worker.close(join_timeout_s=self._args.stop_timeout_s),
            lambda: self._state_reader.close(join_timeout_s=self._args.state_timeout_s),
        ):
            try:
                close()
            except BaseException as exc:
                close_errors.append(exc)
        if close_errors:
            failure = RuntimeError(f"HARD STOP FAILURE: {close_errors[0]}; use the external E-stop")
            failure.__cause__ = close_errors[0]
            self._record_termination_failure(failure)
        self.raise_if_failed()


@dataclass(frozen=True)
class InferenceSnapshot:
    last_fresh_result_s: float
    latency_s: float | None
    delay_steps: int
    queue_steps: int
    rejected_stale: bool


class InferenceStatus:
    def __init__(self, *, initialized_at_s: float) -> None:
        self._lock = threading.Lock()
        self._snapshot = InferenceSnapshot(
            last_fresh_result_s=initialized_at_s,
            latency_s=None,
            delay_steps=0,
            queue_steps=0,
            rejected_stale=False,
        )

    def mark_result(
        self,
        *,
        now_s: float,
        latency_s: float,
        delay_steps: int,
        queue_steps: int,
        rejected_stale: bool,
    ) -> None:
        with self._lock:
            last_fresh = self._snapshot.last_fresh_result_s
            if queue_steps > 0 and not rejected_stale:
                last_fresh = now_s
            self._snapshot = InferenceSnapshot(
                last_fresh,
                latency_s,
                delay_steps,
                queue_steps,
                rejected_stale,
            )

    def snapshot(self) -> InferenceSnapshot:
        with self._lock:
            return self._snapshot


class FrankaDeployment:
    def __init__(self, args: argparse.Namespace) -> None:
        from openpi_client import websocket_client_policy

        self._args = args
        self._stop = threading.Event()
        self._errors: queue.Queue[BaseException] = queue.Queue()
        self._policy_client_type = websocket_client_policy.WebsocketClientPolicy
        self._warmup_policy: PolicyLike | None = None
        self._policy: PolicyLike | None = None
        self._infer_thread: threading.Thread | None = None
        robot: PolymetisPanda | None = None
        exterior: RGBZmqCamera | None = None
        wrist: RGBZmqCamera | None = None
        try:
            self._warmup_policy = self._connect_policy(args.warmup_timeout_s)
            self._metadata = _require_expected_metadata(self._warmup_policy.get_server_metadata())
            robot = PolymetisPanda(args)
            exterior = RGBZmqCamera(
                role="exterior",
                host=args.camera_host,
                port=args.exterior_camera_port,
                timeout_ms=args.camera_timeout_ms,
            )
            wrist = RGBZmqCamera(
                role="wrist",
                host=args.camera_host,
                port=args.wrist_camera_port,
                timeout_ms=args.camera_timeout_ms,
            )
        except BaseException:
            for resource in (wrist, exterior, robot, self._warmup_policy):
                if resource is not None:
                    with contextlib.suppress(BaseException):
                        resource.close()
            self._warmup_policy = None
            raise
        self._robot = robot
        self._exterior = exterior
        self._wrist = wrist
        self._chunks = ActionChunkQueue(
            ChunkQueueConfig(
                max_queue_size=args.max_queue_size,
                max_overlap_steps=args.max_overlap_steps,
                freshness_ttl_s=args.action_ttl_s,
            )
        )
        self._safety = PandaSafetyFilter(
            SafetyConfig(
                max_joint_step_rad=args.max_joint_step_rad,
                max_joint_acceleration_rad_s2=args.max_joint_acceleration_rad_s2,
            )
        )
        self._status: InferenceStatus | None = None
        self._heartbeat_lock = threading.Lock()
        self._last_control_heartbeat_s = 0.0
        self._control_watchdog_thread: threading.Thread | None = None
        self._stopper_thread = threading.Thread(target=self._stopper_loop, name="stop-latch", daemon=True)
        self._stopper_thread.start()

    def _connect_policy(self, receive_timeout_s: float) -> PolicyLike:
        return self._policy_client_type(
            self._args.policy_host,
            self._args.policy_port,
            connect_timeout_s=self._args.policy_connect_timeout_s,
            receive_timeout_s=receive_timeout_s,
        )

    def request_stop(self) -> None:
        self._stop.set()

    def _stopper_loop(self) -> None:
        self._stop.wait()
        try:
            self._robot.stop("stop latch set")
            self._robot.raise_if_failed()
        except BaseException as exc:
            self._errors.put(exc)

    def _raise_if_cancelled(self, stage: str) -> None:
        if self._stop.is_set():
            self._robot.stop(f"cancelled {stage}")
            raise RuntimeError(f"shutdown requested {stage}")

    def _touch_control_heartbeat(self) -> None:
        with self._heartbeat_lock:
            self._last_control_heartbeat_s = time.monotonic()

    def _control_watchdog_loop(self) -> None:
        interval_s = min(0.05, self._args.control_loop_watchdog_timeout_s / 4.0)
        while not self._stop.wait(interval_s):
            with self._heartbeat_lock:
                last_heartbeat_s = self._last_control_heartbeat_s
            age_s = time.monotonic() - last_heartbeat_s
            if self._robot.armed and age_s >= self._args.control_loop_watchdog_timeout_s:
                reason = f"control-loop heartbeat expired ({age_s:.3f}s)"
                self._robot.stop(reason)
                self._errors.put(TimeoutError(reason))
                self._stop.set()
                return

    def _capture(self) -> tuple[dict[str, Any], float]:
        observed_at_s = time.monotonic()
        exterior = self._exterior.read()
        wrist = self._wrist.read()
        state = self._robot.read_state()
        observation = _build_policy_observation(
            exterior_rgb=exterior,
            wrist_rgb=wrist,
            state=state,
            prompt=self._args.prompt,
        )
        return observation, observed_at_s

    def _infer(self, policy: PolicyLike, observation: dict[str, Any]) -> np.ndarray:
        return _request_action_chunk(
            policy,
            observation,
            expected_horizon=self._metadata.action_horizon,
        )

    def _inference_loop(self) -> None:
        assert self._status is not None
        assert self._policy is not None
        control_hz = self._metadata.control_hz
        try:
            while not self._stop.is_set():
                if self._chunks.qsize() > self._args.queue_refill_threshold:
                    self._stop.wait(0.005)
                    continue
                observation, observed_at_s = self._capture()
                chunk = self._infer(self._policy, observation)
                now_s = time.monotonic()
                latency_s = now_s - observed_at_s
                delay_steps = max(0, int(latency_s * control_hz))
                update = self._chunks.enqueue(
                    chunk,
                    delay_steps=delay_steps,
                    observed_at_s=observed_at_s,
                    now_s=now_s,
                )
                self._status.mark_result(
                    now_s=now_s,
                    latency_s=latency_s,
                    delay_steps=delay_steps,
                    queue_steps=update.accepted_steps,
                    rejected_stale=update.rejected_stale,
                )
        except BaseException as exc:
            self._errors.put(exc)
            self._stop.set()

    def _raise_background_error(self) -> None:
        try:
            error = self._errors.get_nowait()
        except queue.Empty:
            return
        raise RuntimeError("asynchronous inference failed") from error

    def _prepare_policy_sessions(self) -> None:
        # Warmup validates the complete observation/response path before
        # arming. Its returned action chunk is deliberately not enqueued.
        self._raise_if_cancelled("before policy warmup")
        print("[warmup] requesting one action chunk (result will be discarded)", flush=True)
        warmup_policy = self._warmup_policy
        if warmup_policy is None:
            raise RuntimeError("warmup policy session is not available")
        try:
            warmup_observation, _ = self._capture()
            self._raise_if_cancelled("after warmup capture")
            self._infer(warmup_policy, warmup_observation)
            self._raise_if_cancelled("after policy warmup")
        finally:
            warmup_policy.close()
            self._warmup_policy = None
        print("[warmup] policy contract passed; action chunk discarded", flush=True)

        # The long warmup deadline must not weaken live control. Reconnect
        # with the short inference timeout and repeat the exact handshake.
        self._raise_if_cancelled("before realtime policy reconnect")
        self._policy = self._connect_policy(self._args.inference_timeout_s)
        self._raise_if_cancelled("after realtime policy reconnect")
        self._metadata = _require_expected_metadata(self._policy.get_server_metadata())
        self._raise_if_cancelled("after realtime metadata handshake")
        print("[policy] realtime session connected and metadata revalidated", flush=True)

    def run(self) -> None:
        self._prepare_policy_sessions()
        self._raise_if_cancelled("before arming")

        if self._args.enable_motion:
            self._robot.arm(self._stop)
            self._raise_if_cancelled("after arming")
        else:
            print("[shadow] motion disabled; no arm or gripper commands will be sent", flush=True)

        initialized_at_s = time.monotonic()
        self._status = InferenceStatus(initialized_at_s=initialized_at_s)
        self._touch_control_heartbeat()
        if self._robot.armed:
            self._control_watchdog_thread = threading.Thread(
                target=self._control_watchdog_loop,
                name="control-loop-watchdog",
                daemon=True,
            )
            self._control_watchdog_thread.start()
            print("[armed] joint impedance started; automatic recovery is disabled", flush=True)
        self._infer_thread = threading.Thread(target=self._inference_loop, name="policy-inference", daemon=True)
        self._infer_thread.start()

        period_s = 1.0 / self._metadata.control_hz
        deadline_s = time.monotonic()
        started_s = deadline_s
        last_log_s = 0.0
        stop_reason = "operator or duration stop"
        try:
            while True:
                self._raise_background_error()
                if self._stop.is_set():
                    break
                self._touch_control_heartbeat()
                now_s = time.monotonic()
                if self._args.duration_s > 0 and now_s - started_s >= self._args.duration_s:
                    break
                self._robot.raise_if_failed()

                status = self._status.snapshot()
                freshness_age_s = now_s - status.last_fresh_result_s
                if freshness_age_s >= self._args.watchdog_timeout_s:
                    stop_reason = f"policy freshness watchdog expired ({freshness_age_s:.3f}s)"
                    self._robot.stop(stop_reason)
                    raise RuntimeError(stop_reason)

                measured = self._robot.read_state()
                policy_action = self._chunks.pop(now_s=now_s)
                safe_action = self._safety.filter(policy_action, measured, dt_s=period_s)

                if self._robot.armed:
                    self._robot.command_arm(safe_action[:7])
                    self._robot.command_gripper(float(safe_action[7]), now_s=now_s)

                if now_s - last_log_s >= 1.0:
                    latency = "n/a" if status.latency_s is None else f"{status.latency_s * 1000.0:.0f}ms"
                    mode = "ARMED" if self._robot.armed else "SHADOW"
                    source = "policy" if policy_action is not None else "measured-hold"
                    print(
                        f"[{mode}] source={source} queue={self._chunks.qsize(now_s=now_s)} "
                        f"infer={latency} delay={status.delay_steps} freshness={freshness_age_s:.2f}s",
                        flush=True,
                    )
                    last_log_s = now_s

                deadline_s += period_s
                remaining_s = deadline_s - time.monotonic()
                if remaining_s > 0:
                    self._stop.wait(remaining_s)
                else:
                    deadline_s = time.monotonic()
        except BaseException as exc:
            stop_reason = f"runtime fault: {type(exc).__name__}: {exc}"
            raise
        finally:
            self._stop.set()
            self._chunks.clear()
            try:
                self._robot.stop(stop_reason)
            finally:
                if self._control_watchdog_thread is not None:
                    self._control_watchdog_thread.join(timeout=0.2)
                shutdown_wait_s = 2.0 * self._args.camera_timeout_ms / 1000.0
                shutdown_wait_s += self._args.inference_timeout_s + self._args.state_timeout_s + 1.0
                self._infer_thread.join(timeout=shutdown_wait_s)
                if self._infer_thread.is_alive():
                    raise RuntimeError("inference thread exceeded all configured RPC deadlines during shutdown")
        self._raise_background_error()
        self._robot.raise_if_failed()

    def close(self) -> None:
        self._stop.set()
        self._stopper_thread.join(timeout=self._args.stop_timeout_s + 0.2)
        first_error: BaseException | None = None
        if self._stopper_thread.is_alive():
            first_error = RuntimeError("stop-latch thread did not finish its bounded terminate attempt")
        if self._infer_thread is not None and self._infer_thread.is_alive():
            print(
                "[shutdown] inference thread exceeded its bounded waits; motion is stopped, "
                "leaving read-only clients for process teardown",
                flush=True,
            )
            if first_error is None:
                first_error = RuntimeError("inference thread remains alive during cleanup")
        cleanup = [self._robot.close, self._exterior.close, self._wrist.close]
        if self._warmup_policy is not None:
            cleanup.append(self._warmup_policy.close)
            self._warmup_policy = None
        if self._policy is not None:
            cleanup.append(self._policy.close)
            self._policy = None
        for close in cleanup:
            try:
                close()
            except BaseException as exc:
                if first_error is None:
                    first_error = exc
        if first_error is not None:
            raise RuntimeError("deployment cleanup failed") from first_error


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prompt", required=True)
    parser.add_argument("--policy-host", default="127.0.0.1")
    parser.add_argument("--policy-port", type=int, default=8000)
    parser.add_argument("--policy-connect-timeout-s", type=float, default=15.0)
    parser.add_argument("--warmup-timeout-s", type=float, default=120.0)
    parser.add_argument("--inference-timeout-s", type=float, default=5.0)
    parser.add_argument("--allow-remote-policy", action="store_true")

    parser.add_argument("--polymetis-host", default="127.0.0.1")
    parser.add_argument("--arm-port", type=int, default=50051)
    parser.add_argument("--gripper-port", type=int, default=50052)
    parser.add_argument("--allow-remote-polymetis", action="store_true")
    parser.add_argument("--state-poll-hz", type=float, default=50.0)
    parser.add_argument("--state-timeout-s", type=float, default=0.5)
    parser.add_argument("--command-watchdog-timeout-s", type=float, default=0.25)
    parser.add_argument("--control-loop-watchdog-timeout-s", type=float, default=0.25)
    parser.add_argument("--arm-timeout-s", type=float, default=5.0)
    parser.add_argument("--stop-timeout-s", type=float, default=1.0)

    parser.add_argument("--camera-host", default="127.0.0.1")
    parser.add_argument("--exterior-camera-port", type=int, required=True)
    parser.add_argument("--wrist-camera-port", type=int, required=True)
    parser.add_argument("--camera-timeout-ms", type=int, default=1000)
    parser.add_argument("--allow-remote-camera-pickle", action="store_true")

    parser.add_argument("--duration-s", type=float, default=0.0, help="0 means run until interrupted")
    parser.add_argument("--queue-refill-threshold", type=int, default=5)
    parser.add_argument("--max-queue-size", type=int, default=24)
    parser.add_argument("--max-overlap-steps", type=int, default=4)
    parser.add_argument("--action-ttl-s", type=float, default=1.25)
    parser.add_argument("--watchdog-timeout-s", type=float, default=2.5)

    parser.add_argument("--max-joint-step-rad", type=float, default=0.025)
    parser.add_argument("--max-joint-acceleration-rad-s2", type=float, default=3.0)
    parser.add_argument("--gripper-deadband", type=float, default=0.05)
    parser.add_argument("--gripper-min-interval-s", type=float, default=0.25)
    parser.add_argument("--gripper-speed", type=float, default=0.05)
    parser.add_argument("--gripper-force", type=float, default=20.0)
    parser.add_argument("--enable-motion", action="store_true")
    return parser.parse_args()


def _validate_args(args: argparse.Namespace) -> None:
    if not args.prompt.strip():
        raise ValueError("--prompt must not be empty")
    if args.exterior_camera_port == args.wrist_camera_port:
        raise ValueError("exterior and wrist camera ports must be different")
    for name in ("policy_port", "arm_port", "gripper_port", "exterior_camera_port", "wrist_camera_port"):
        value = getattr(args, name)
        if not 1 <= value <= 65535:
            raise ValueError(f"--{name.replace('_', '-')} must be in [1, 65535]")
    positive = (
        "policy_connect_timeout_s",
        "warmup_timeout_s",
        "inference_timeout_s",
        "camera_timeout_ms",
        "state_poll_hz",
        "state_timeout_s",
        "command_watchdog_timeout_s",
        "control_loop_watchdog_timeout_s",
        "arm_timeout_s",
        "stop_timeout_s",
        "max_queue_size",
        "action_ttl_s",
        "watchdog_timeout_s",
        "max_joint_step_rad",
        "max_joint_acceleration_rad_s2",
        "gripper_min_interval_s",
        "gripper_speed",
        "gripper_force",
    )
    for name in positive:
        if getattr(args, name) <= 0:
            raise ValueError(f"--{name.replace('_', '-')} must be > 0")
    if args.queue_refill_threshold < 0 or args.max_overlap_steps < 0:
        raise ValueError("queue refill threshold and max overlap steps must be non-negative")
    if args.queue_refill_threshold >= args.max_queue_size:
        raise ValueError("--queue-refill-threshold must be smaller than --max-queue-size")
    if args.watchdog_timeout_s <= args.action_ttl_s:
        raise ValueError("--watchdog-timeout-s must be greater than --action-ttl-s")
    if not 0.0 <= args.gripper_deadband <= 1.0:
        raise ValueError("--gripper-deadband must be in [0, 1]")
    if args.duration_s < 0:
        raise ValueError("--duration-s must be >= 0")
    if not _is_loopback(args.policy_host) and not args.allow_remote_policy:
        raise ValueError("plaintext policy websocket must use loopback (prefer an SSH tunnel), or opt in explicitly")
    if not _is_loopback(args.polymetis_host) and not args.allow_remote_polymetis:
        raise ValueError("Polymetis must use loopback, or opt in explicitly for a trusted private control network")
    if not _is_loopback(args.camera_host) and not args.allow_remote_camera_pickle:
        raise ValueError("pickle camera RPC must use loopback, or opt in explicitly for a trusted private network")


def main() -> None:
    args = parse_args()
    _validate_args(args)
    runtime: FrankaDeployment | None = None
    shutdown_requested = threading.Event()

    def request_stop() -> None:
        shutdown_requested.set()
        if runtime is not None:
            runtime.request_stop()

    shutdown = TwoStageShutdown(request_stop)
    signal.signal(signal.SIGINT, shutdown.handle_signal)
    signal.signal(signal.SIGTERM, shutdown.handle_signal)
    try:
        runtime = FrankaDeployment(args)
        runtime.run()
    finally:
        shutdown.begin_cleanup()
        if runtime is not None:
            runtime.close()


if __name__ == "__main__":
    main()
