#!/usr/bin/env python3
"""Fail-closed GELLO-compatible ZMQ bridge for a Polymetis Panda service.

Unlike GELLO's historical Panda adapter, this bridge never calls ``go_home``,
does not perform automatic error recovery, binds to loopback by default, and
stops the active impedance policy when its command watchdog expires.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable
import contextlib
import ipaddress
import pickle
import queue
import signal
import threading
import time

from franka_runtime import TwoStageShutdown
import numpy as np
import zmq

MAX_OPEN_M = 0.08
CONTROL_HZ = 20.0
JOINT_LOW = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
JOINT_HIGH = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
JOINT_MARGIN_RAD = 0.02


def _numpy(value) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    return np.asarray(value, dtype=np.float64)


def _validated_command_velocity(
    target_q: np.ndarray,
    measured_q: np.ndarray,
    previous_velocity: np.ndarray,
    *,
    dt_s: float,
    max_acceleration_rad_s2: float,
) -> np.ndarray:
    if not np.isfinite(dt_s) or dt_s <= 0.0:
        raise ValueError("command interval must be finite and positive")
    velocity = (target_q - measured_q) / dt_s
    max_velocity_change = max_acceleration_rad_s2 * dt_s
    if np.any(np.abs(velocity - previous_velocity) > max_velocity_change + 1e-9):
        raise ValueError("joint target exceeds --max-joint-acceleration-rad-s2")
    return velocity


def _is_loopback(host: str) -> bool:
    if host.lower() == "localhost":
        return True
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False


class _LatestGripperWorker:
    """Latest-wins gripper worker with an independently bounded stop RPC."""

    def __init__(self, gripper) -> None:
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
        # non-daemon. A second OS signal is the explicit escape after the
        # external E-stop has been applied.
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
                        self._start_stop_attempt_locked()
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


class _ArmCommandWorker:
    """Serialize legacy arm RPCs and invalidate late completions by epoch."""

    def __init__(
        self,
        robot,
        is_active: Callable[[int], bool],
        on_late_completion: Callable[[str], None],
    ) -> None:
        self._robot = robot
        self._is_active = is_active
        self._on_late_completion = on_late_completion
        self._commands: queue.Queue[tuple[np.ndarray, int, threading.Event, list[BaseException]]] = queue.Queue(
            maxsize=1
        )
        self._stop = threading.Event()
        self._lock = threading.Lock()
        # Keep the process alive until a late arm RPC can issue its compensating
        # terminate. A second OS signal is the explicit escape for a permanent
        # transport hang while the external E-stop is held.
        self._thread = threading.Thread(target=self._run, name="arm-command", daemon=False)
        self._thread.start()

    def submit(self, target: np.ndarray, *, epoch: int, timeout_s: float) -> None:
        completed = threading.Event()
        errors: list[BaseException] = []
        with self._lock:
            if self._stop.is_set():
                raise RuntimeError("arm command worker is stopped")
            try:
                self._commands.put_nowait((np.asarray(target, dtype=np.float64).copy(), epoch, completed, errors))
            except queue.Full as exc:
                raise RuntimeError("previous arm command has not been consumed") from exc
        if not completed.wait(timeout_s):
            raise TimeoutError("arm update RPC did not complete before its deadline")
        if errors:
            raise RuntimeError("arm update RPC failed or completed after stop") from errors[0]

    def _run(self) -> None:
        import torch

        while not self._stop.is_set():
            try:
                target, epoch, completed, errors = self._commands.get(timeout=0.05)
            except queue.Empty:
                continue
            request_started = False
            late_completion = False
            try:
                if self._stop.is_set() or not self._is_active(epoch):
                    raise RuntimeError("arm command invalidated before send")
                # stop() may race immediately after this check. A completion
                # after epoch invalidation triggers another bounded terminate.
                request_started = True
                self._robot.update_desired_joint_positions(torch.as_tensor(target, dtype=torch.float32))
                if self._stop.is_set() or not self._is_active(epoch):
                    late_completion = True
                    raise RuntimeError("arm command completed after the policy was stopped")
            except BaseException as exc:
                errors.append(exc)
            finally:
                completed.set()
                if request_started and (late_completion or self._stop.is_set() or not self._is_active(epoch)):
                    self._on_late_completion("arm update completed after stop")

    def stop(self) -> None:
        with self._lock:
            self._stop.set()
            while True:
                try:
                    _target, _epoch, completed, errors = self._commands.get_nowait()
                except queue.Empty:
                    break
                errors.append(RuntimeError("arm command cancelled by stop"))
                completed.set()

    def close(self, *, join_timeout_s: float = 0.1) -> None:
        self.stop()
        self._thread.join(timeout=join_timeout_s)
        if self._thread.is_alive():
            raise RuntimeError("arm update RPC remains in flight; use the external E-stop")


class SafePanda:
    def __init__(self, args: argparse.Namespace):
        from polymetis import GripperInterface
        from polymetis import RobotInterface

        self._args = args
        self._robot = RobotInterface(ip_address=args.polymetis_host, port=args.arm_port)
        self._gripper = GripperInterface(ip_address=args.polymetis_host, port=args.gripper_port)
        self._armed = False
        self._arming = False
        self._stopped = False
        self._command_epoch = 0
        self._armed_at: float | None = None
        self._last_command_time: float | None = None
        self._previous_command_velocity = np.zeros(7, dtype=np.float64)
        self._last_gripper_closed: float | None = None
        self._state_lock = threading.Lock()
        self._watchdog_stop = threading.Event()
        self._stop_complete = threading.Event()
        self._arming_thread: threading.Thread | None = None
        self._termination_threads: list[threading.Thread] = []
        self._termination_threads_lock = threading.Lock()
        self._fatal_error: BaseException | None = None
        self._fatal_lock = threading.Lock()
        self._arm_worker = _ArmCommandWorker(self._robot, self._epoch_is_active, self._terminate_policy_bounded)
        self._gripper_worker = _LatestGripperWorker(self._gripper)
        self._watchdog_thread = threading.Thread(target=self._watchdog_loop, name="command-watchdog", daemon=True)
        self._watchdog_thread.start()

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
            self._record_fatal(failure)
            print(f"[HARD-STOP-FAILURE] {failure}", flush=True)

    def _record_fatal(self, error: BaseException) -> None:
        with self._fatal_lock:
            if self._fatal_error is None:
                self._fatal_error = error

    def _wait_for_gripper_stop_bounded(self, reason: str) -> None:
        try:
            self._gripper_worker.wait_for_stop(timeout_s=self._args.stop_timeout_s)
        except BaseException as exc:
            failure = RuntimeError(f"HARD STOP FAILURE: {reason}; {exc}; use the external E-stop")
            failure.__cause__ = exc
            self._record_fatal(failure)
            print(f"[HARD-STOP-FAILURE] {failure}", flush=True)

    def _epoch_is_active(self, epoch: int) -> bool:
        with self._state_lock:
            return self._armed and not self._stopped and epoch == self._command_epoch

    def _watchdog_loop(self) -> None:
        interval_s = min(0.05, self._args.watchdog_timeout_s / 4.0)
        while not self._watchdog_stop.wait(interval_s):
            try:
                self.check_watchdog()
            except BaseException as exc:
                self._record_fatal(exc)
                return

    def raise_if_failed(self) -> None:
        self._gripper_worker.raise_if_failed()
        with self._fatal_lock:
            error = self._fatal_error
        if error is not None:
            raise RuntimeError("robot bridge failed closed") from error

    def arm(self, stop_event: threading.Event) -> None:
        with self._state_lock:
            if self._stopped:
                raise RuntimeError("a stopped bridge cannot be armed")
        if input("Clear the workspace, hold the external activation device, then type ARM: ").strip() != "ARM":
            raise RuntimeError("arming cancelled")
        if stop_event.is_set():
            raise RuntimeError("arming cancelled by shutdown request")
        with self._state_lock:
            if self._stopped:
                raise RuntimeError("bridge stopped while waiting for ARM confirmation")
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
                # A timed-out request can still reach the server after the
                # caller has failed closed. Terminate again after that late
                # response so it cannot leave a policy running unattended.
                if abandoned.is_set():
                    self._terminate_policy_bounded("late arming response")

        # Keep the process alive until a late start response has triggered its
        # compensating termination. A second OS signal remains the explicit
        # operator escape hatch for an RPC that never returns.
        start_thread = threading.Thread(target=start_policy, name="start-joint-impedance", daemon=False)
        self._arming_thread = start_thread
        try:
            start_thread.start()
            deadline = time.monotonic() + self._args.arm_timeout_s
            while not completed.wait(min(0.05, max(0.0, deadline - time.monotonic()))):
                if stop_event.is_set():
                    raise RuntimeError("arming interrupted by shutdown")
                if time.monotonic() >= deadline:
                    raise TimeoutError("start_joint_impedance timed out")
            if errors:
                raise RuntimeError("start_joint_impedance failed") from errors[0]
            with self._state_lock:
                if self._stopped or stop_event.is_set():
                    abandoned.set()
                    raise RuntimeError("bridge stopped while arming")
                self._arming = False
                self._armed = True
                self._command_epoch += 1
                self._armed_at = time.monotonic()
                self._last_command_time = None
                self._previous_command_velocity.fill(0.0)
        except BaseException:
            # The request may have reached the server even when its response
            # was lost. Always attempt termination before failing arming.
            abandoned.set()
            self.stop("arming failed")
            if start_thread.ident is not None and not completed.is_set():
                self._record_fatal(
                    RuntimeError(
                        "HARD STOP FAILURE: start_joint_impedance remains in flight after arming failed; "
                        "policy state is unknown; use the external E-stop"
                    )
                )
            raise
        print("[armed] joint impedance active; no automatic recovery is enabled", flush=True)

    def num_dofs(self) -> int:
        return 8

    def _gripper_closed_fraction(self) -> float:
        width = float(self._gripper.get_state().width)
        if not np.isfinite(width) or not 0.0 <= width <= MAX_OPEN_M:
            raise RuntimeError("measured gripper width is invalid or outside [0, 0.08] m")
        return 1.0 - width / MAX_OPEN_M

    def get_joint_state(self) -> np.ndarray:
        q = _numpy(self._robot.get_joint_positions()).reshape(7)
        if not np.isfinite(q).all() or np.any(q < JOINT_LOW) or np.any(q > JOINT_HIGH):
            raise RuntimeError("measured Panda joint position is invalid or outside limits")
        return np.concatenate([q, [self._gripper_closed_fraction()]])

    def get_observations(self) -> dict[str, np.ndarray]:
        q = _numpy(self._robot.get_joint_positions()).reshape(7)
        dq = _numpy(self._robot.get_joint_velocities()).reshape(7)
        position, quaternion = self._robot.get_ee_pose()
        ee_pose = np.concatenate([_numpy(position).reshape(3), _numpy(quaternion).reshape(4)])
        gripper_closed = self._gripper_closed_fraction()
        if (
            not np.isfinite(q).all()
            or not np.isfinite(dq).all()
            or not np.isfinite(ee_pose).all()
            or np.any(q < JOINT_LOW)
            or np.any(q > JOINT_HIGH)
        ):
            raise RuntimeError("measured Panda state is invalid or outside limits")
        return {
            "joint_positions": np.concatenate([q, [gripper_closed]]),
            "joint_velocities": np.concatenate([dq, [0.0]]),
            "ee_pos_quat": ee_pose,
            "gripper_position": np.array([gripper_closed], dtype=np.float64),
        }

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        with self._state_lock:
            if not self._armed or self._stopped:
                raise RuntimeError("robot bridge is not armed")
            last_command_time = self._last_command_time
            previous_command_velocity = self._previous_command_velocity.copy()
        command = np.asarray(joint_state, dtype=np.float64).reshape(-1)
        if command.shape != (8,) or not np.isfinite(command).all():
            self.stop("invalid command")
            raise ValueError(f"expected 8 finite command values, got {command.shape}")
        current = _numpy(self._robot.get_joint_positions()).reshape(7)
        if not np.isfinite(current).all() or np.any(current < JOINT_LOW) or np.any(current > JOINT_HIGH):
            self.stop("invalid measured joint state")
            raise RuntimeError("measured Panda joint position is invalid or outside limits")
        if np.any(command[:7] < JOINT_LOW + JOINT_MARGIN_RAD) or np.any(command[:7] > JOINT_HIGH - JOINT_MARGIN_RAD):
            self.stop("joint limit violation")
            raise ValueError("joint target is outside conservative Panda limits")
        if not 0.0 <= command[7] <= 1.0:
            self.stop("gripper range violation")
            raise ValueError("gripper_closed_fraction must be in [0, 1]")
        if np.max(np.abs(command[:7] - current)) > self._args.max_joint_step_rad:
            self.stop("joint step violation")
            raise ValueError("joint target exceeds --max-joint-step-rad")
        command_time = time.monotonic()
        if last_command_time is not None and command_time - last_command_time > self._args.watchdog_timeout_s:
            self.stop("stale command interval")
            raise RuntimeError("command arrived after the watchdog deadline; bridge must be restarted and re-armed")
        command_interval_s = 1.0 / CONTROL_HZ if last_command_time is None else command_time - last_command_time
        try:
            command_velocity = _validated_command_velocity(
                command[:7],
                current,
                previous_command_velocity,
                dt_s=command_interval_s,
                max_acceleration_rad_s2=self._args.max_joint_acceleration_rad_s2,
            )
        except ValueError:
            self.stop("joint acceleration violation")
            raise

        # get_joint_positions is an unbounded legacy RPC. The independent
        # watchdog may have stopped the bridge while that call was blocked.
        with self._state_lock:
            if not self._armed or self._stopped:
                raise RuntimeError("robot bridge stopped while reading state")
            epoch = self._command_epoch

        self._arm_worker.submit(command[:7], epoch=epoch, timeout_s=self._args.arm_command_timeout_s)
        # Do not refresh the watchdog or enqueue a gripper command if the
        # independent watchdog stopped the policy during the arm RPC.
        with self._state_lock:
            if not self._armed or self._stopped or epoch != self._command_epoch:
                raise RuntimeError("robot bridge stopped during arm command")
            self._last_command_time = time.monotonic()
            self._previous_command_velocity = command_velocity

        gripper_closed = float(command[7])
        if self._last_gripper_closed is None or abs(gripper_closed - self._last_gripper_closed) >= 0.05:
            width = (1.0 - gripper_closed) * MAX_OPEN_M
            self._gripper_worker.submit(
                width=width,
                speed=self._args.gripper_speed,
                force=self._args.gripper_force,
            )
            self._last_gripper_closed = gripper_closed

    def check_watchdog(self) -> None:
        with self._state_lock:
            now = time.monotonic()
            if self._armed and self._last_command_time is None:
                expired = self._armed_at is None or now - self._armed_at > self._args.first_command_timeout_s
                reason = "first command watchdog expired"
            else:
                expired = (
                    self._armed
                    and self._last_command_time is not None
                    and now - self._last_command_time > self._args.watchdog_timeout_s
                )
                reason = "command watchdog expired"
        if expired:
            self.stop(reason)
            raise RuntimeError(f"{reason}; bridge must be restarted and re-armed")

    def stop(self, reason: str) -> None:
        with self._state_lock:
            first_stop = not self._stopped
            self._stopped = True
            was_active = self._armed or self._arming
            self._armed = False
            self._arming = False
            self._command_epoch += 1
        self._arm_worker.stop()
        gripper_stop_started = self._gripper_worker.stop(ensure_hardware_stop=was_active)
        if first_stop:
            try:
                if was_active:
                    self._terminate_policy_bounded(reason)
                if gripper_stop_started:
                    self._wait_for_gripper_stop_bounded(reason)
            finally:
                self._stop_complete.set()
            print(f"[stopped] {reason}; restart and re-arm manually", flush=True)
        elif not self._stop_complete.wait(self._args.stop_timeout_s + 0.5):
            self._record_fatal(
                RuntimeError(
                    "HARD STOP FAILURE: concurrent stop did not complete; policy state is unknown; "
                    "use the external E-stop"
                )
            )

    def close(self) -> None:
        self._watchdog_stop.set()
        self.stop("bridge shutdown")
        self._watchdog_thread.join(timeout=self._args.stop_timeout_s + 0.5)
        if self._watchdog_thread.is_alive():
            self._record_fatal(RuntimeError("watchdog thread did not stop"))
        for close in (
            lambda: self._arm_worker.close(join_timeout_s=self._args.stop_timeout_s),
            lambda: self._gripper_worker.close(join_timeout_s=self._args.stop_timeout_s),
        ):
            try:
                close()
            except BaseException as exc:
                self._record_fatal(exc)
        self.raise_if_failed()


def _serve(robot: SafePanda, bind_host: str, port: int, stop_event: threading.Event) -> None:
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.setsockopt(zmq.RCVTIMEO, 50)
    socket.setsockopt(zmq.LINGER, 0)
    socket.bind(f"tcp://{bind_host}:{port}")
    print(f"[server] tcp://{bind_host}:{port}", flush=True)
    try:
        while not stop_event.is_set():
            robot.raise_if_failed()
            robot.check_watchdog()
            fatal_request_error: BaseException | None = None
            method = None
            try:
                request = pickle.loads(socket.recv())
            except zmq.Again:
                continue
            try:
                method = request.get("method")
                arguments = request.get("args", {})
                if method == "num_dofs":
                    result = robot.num_dofs()
                elif method == "get_joint_state":
                    result = robot.get_joint_state()
                elif method == "get_observations":
                    result = robot.get_observations()
                elif method == "command_joint_state":
                    result = robot.command_joint_state(**arguments)
                elif method == "stop":
                    robot.stop(**arguments)
                    robot.raise_if_failed()
                    result = None
                else:
                    raise ValueError(f"unsupported method: {method}")
            except BaseException as exc:
                result = {"error": f"{type(exc).__name__}: {exc}"}
                if method in {"command_joint_state", "get_joint_state", "get_observations", "stop"}:
                    robot.stop("hardware RPC failed")
                    fatal_request_error = exc
            socket.send(pickle.dumps(result, protocol=pickle.HIGHEST_PROTOCOL))
            if fatal_request_error is not None:
                raise RuntimeError("hardware RPC failed; bridge stopped") from fatal_request_error
    finally:
        socket.close()
        context.term()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--polymetis-host", default="127.0.0.1")
    parser.add_argument("--arm-port", type=int, default=50051)
    parser.add_argument("--gripper-port", type=int, default=50052)
    parser.add_argument("--bind-host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=6001)
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--allow-non-loopback", action="store_true")
    parser.add_argument("--allow-remote-polymetis", action="store_true")
    parser.add_argument("--watchdog-timeout-s", type=float, default=0.25)
    parser.add_argument("--first-command-timeout-s", type=float, default=10.0)
    parser.add_argument("--arm-timeout-s", type=float, default=5.0)
    parser.add_argument("--arm-command-timeout-s", type=float, default=0.2)
    parser.add_argument("--stop-timeout-s", type=float, default=1.0)
    parser.add_argument("--max-joint-step-rad", type=float, default=0.025)
    parser.add_argument("--max-joint-acceleration-rad-s2", type=float, default=3.0)
    parser.add_argument("--gripper-speed", type=float, default=0.05)
    parser.add_argument("--gripper-force", type=float, default=20.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not _is_loopback(args.bind_host) and not args.allow_non_loopback:
        raise ValueError("non-loopback pickle RPC requires explicit --allow-non-loopback and a trusted private network")
    if not _is_loopback(args.polymetis_host) and not args.allow_remote_polymetis:
        raise ValueError("remote Polymetis requires --allow-remote-polymetis on a trusted private control network")
    if (
        args.watchdog_timeout_s <= 0
        or args.first_command_timeout_s <= 0
        or args.arm_timeout_s <= 0
        or args.arm_command_timeout_s <= 0
        or args.stop_timeout_s <= 0
        or args.max_joint_step_rad <= 0
        or args.max_joint_acceleration_rad_s2 <= 0
    ):
        raise ValueError("watchdog, first-command, arm/command, stop timeouts and joint safety limits must be positive")
    if args.gripper_speed <= 0 or args.gripper_force <= 0:
        raise ValueError("gripper speed and force must be positive")
    for name in ("arm_port", "gripper_port", "port"):
        port = getattr(args, name)
        if not 1 <= port <= 65535:
            raise ValueError(f"--{name.replace('_', '-')} must be in [1, 65535]")

    stop_event = threading.Event()

    def request_stop() -> None:
        stop_event.set()

    shutdown = TwoStageShutdown(request_stop)
    signal.signal(signal.SIGINT, shutdown.handle_signal)
    signal.signal(signal.SIGTERM, shutdown.handle_signal)
    robot: SafePanda | None = None
    try:
        robot = SafePanda(args)
        if args.enable_motion:
            robot.arm(stop_event)
        else:
            print("[read-only] omit --enable-motion only for connectivity checks", flush=True)
        if stop_event.is_set():
            raise RuntimeError("shutdown requested before server start")
        _serve(robot, args.bind_host, args.port, stop_event)
    finally:
        shutdown.begin_cleanup()
        if robot is not None:
            robot.close()


if __name__ == "__main__":
    main()
