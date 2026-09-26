from __future__ import annotations

import builtins
import sys

# Safety-worker internals are tested directly without widening the runtime API.
# ruff: noqa: SLF001
import threading
from types import SimpleNamespace

import pytest

from examples.franka_real import panda_zmq_server


def test_bridge_acceleration_gate_rejects_direction_reversal() -> None:
    measured = panda_zmq_server.np.zeros(7)
    first_target = panda_zmq_server.np.full(7, 0.0075)
    first_velocity = panda_zmq_server._validated_command_velocity(
        first_target,
        measured,
        panda_zmq_server.np.zeros(7),
        dt_s=0.05,
        max_acceleration_rad_s2=3.0,
    )
    panda_zmq_server.np.testing.assert_allclose(first_velocity, 0.15)

    with pytest.raises(ValueError, match="max-joint-acceleration"):
        panda_zmq_server._validated_command_velocity(
            measured - 0.0075,
            measured,
            first_velocity,
            dt_s=0.05,
            max_acceleration_rad_s2=3.0,
        )


def _watchdog_fixture(*, armed_at: float, last_command_time: float | None):
    stopped: list[str] = []
    robot = SimpleNamespace(
        _state_lock=threading.Lock(),
        _armed=True,
        _armed_at=armed_at,
        _last_command_time=last_command_time,
        _args=SimpleNamespace(first_command_timeout_s=10.0, watchdog_timeout_s=0.25),
        stop=stopped.append,
    )
    return robot, stopped


def test_first_command_has_separate_startup_grace(monkeypatch: pytest.MonkeyPatch) -> None:
    robot, stopped = _watchdog_fixture(armed_at=100.0, last_command_time=None)
    monkeypatch.setattr(panda_zmq_server.time, "monotonic", lambda: 109.9)

    panda_zmq_server.SafePanda.check_watchdog(robot)

    assert stopped == []


def test_first_command_grace_still_fails_closed(monkeypatch: pytest.MonkeyPatch) -> None:
    robot, stopped = _watchdog_fixture(armed_at=100.0, last_command_time=None)
    monkeypatch.setattr(panda_zmq_server.time, "monotonic", lambda: 110.1)

    with pytest.raises(RuntimeError, match="first command watchdog expired"):
        panda_zmq_server.SafePanda.check_watchdog(robot)

    assert stopped == ["first command watchdog expired"]


def test_steady_state_uses_short_command_watchdog(monkeypatch: pytest.MonkeyPatch) -> None:
    robot, stopped = _watchdog_fixture(armed_at=100.0, last_command_time=120.0)
    monkeypatch.setattr(panda_zmq_server.time, "monotonic", lambda: 120.3)

    with pytest.raises(RuntimeError, match="command watchdog expired"):
        panda_zmq_server.SafePanda.check_watchdog(robot)

    assert stopped == ["command watchdog expired"]


def test_late_arm_update_triggers_second_terminate(monkeypatch: pytest.MonkeyPatch) -> None:
    update_started = threading.Event()
    release_update = threading.Event()
    late_terminate = threading.Event()
    active = True

    class FakeRobot:
        def update_desired_joint_positions(self, _target: object) -> None:
            update_started.set()
            release_update.wait(1.0)

    monkeypatch.setitem(
        sys.modules,
        "torch",
        SimpleNamespace(as_tensor=lambda value, **_kwargs: value, float32=object()),
    )
    worker = panda_zmq_server._ArmCommandWorker(
        FakeRobot(),
        lambda _epoch: active,
        lambda _reason: late_terminate.set(),
    )
    assert not worker._thread.daemon
    submit_error: list[BaseException] = []

    def submit() -> None:
        try:
            worker.submit(panda_zmq_server.np.zeros(7), epoch=1, timeout_s=0.5)
        except BaseException as exc:
            submit_error.append(exc)

    submit_thread = threading.Thread(target=submit)
    try:
        submit_thread.start()
        assert update_started.wait(1.0)
        active = False
        worker.stop()
        release_update.set()
        assert late_terminate.wait(1.0)
        submit_thread.join(timeout=1.0)
        assert submit_error
    finally:
        release_update.set()
        worker.close()
        submit_thread.join(timeout=1.0)


def test_gripper_worker_stops_during_inflight_goto_then_compensates() -> None:
    goto_started = threading.Event()
    release_goto = threading.Event()
    first_stop_called = threading.Event()
    second_stop_called = threading.Event()
    release_second_stop = threading.Event()
    stop_calls = 0

    class FakeGripper:
        def goto(self, **_kwargs: object) -> None:
            goto_started.set()
            release_goto.wait(1.0)

        def stop(self) -> None:
            nonlocal stop_calls
            stop_calls += 1
            if stop_calls == 1:
                first_stop_called.set()
            else:
                second_stop_called.set()
                release_second_stop.wait(1.0)

    worker = panda_zmq_server._LatestGripperWorker(FakeGripper())
    try:
        assert not worker._thread.daemon
        worker.submit(width=0.02, speed=0.05, force=20.0)
        assert goto_started.wait(1.0)
        worker.stop()
        assert first_stop_called.wait(1.0)
        assert not worker._stop_threads[0].daemon
        assert not release_goto.is_set()
        wait_finished = threading.Event()
        wait_errors: list[BaseException] = []

        def wait_for_stop() -> None:
            try:
                worker.wait_for_stop(timeout_s=1.0)
            except BaseException as exc:
                wait_errors.append(exc)
            finally:
                wait_finished.set()

        wait_thread = threading.Thread(target=wait_for_stop)
        wait_thread.start()
        assert not wait_finished.wait(0.02)
        release_goto.set()
        assert second_stop_called.wait(1.0)
        assert not wait_finished.wait(0.02)
        release_second_stop.set()
        assert wait_finished.wait(1.0)
        wait_thread.join(timeout=1.0)
        assert not wait_errors
        assert stop_calls == 2
    finally:
        release_goto.set()
        release_second_stop.set()
        worker.close()


def test_gripper_wait_times_out_while_goto_remains_inflight() -> None:
    goto_started = threading.Event()
    release_goto = threading.Event()

    class FakeGripper:
        def goto(self, **_kwargs: object) -> None:
            goto_started.set()
            release_goto.wait(1.0)

        def stop(self) -> None:
            return None

    worker = panda_zmq_server._LatestGripperWorker(FakeGripper())
    try:
        worker.submit(width=0.02, speed=0.05, force=20.0)
        assert goto_started.wait(1.0)
        worker.stop()
        with pytest.raises(TimeoutError, match="Goto RPC remains in flight"):
            worker.wait_for_stop(timeout_s=0.02)
    finally:
        release_goto.set()
        worker.close(join_timeout_s=1.0)


def test_gripper_wait_does_not_wait_for_idle_command_poll() -> None:
    class FakeGripper:
        def stop(self) -> None:
            return None

    class MustNotJoin:
        def join(self, *, timeout: float) -> None:
            del timeout
            pytest.fail("wait_for_stop must track Goto completion, not worker exit")

        def is_alive(self) -> bool:
            return True

    worker = panda_zmq_server._LatestGripperWorker(FakeGripper())
    command_thread = worker._thread
    try:
        assert worker.stop(ensure_hardware_stop=True)
        worker._thread = MustNotJoin()
        worker.wait_for_stop(timeout_s=0.01)
    finally:
        worker._thread = command_thread
        worker.close(join_timeout_s=1.0)


def test_gripper_stop_timeout_becomes_hard_stop_failure() -> None:
    robot = object.__new__(panda_zmq_server.SafePanda)
    robot._args = SimpleNamespace(stop_timeout_s=0.01)
    robot._fatal_error = None
    robot._fatal_lock = threading.Lock()

    def wait_for_stop(*, timeout_s: float) -> None:
        assert timeout_s == 0.01
        raise TimeoutError("gripper stop RPC timed out")

    robot._gripper_worker = SimpleNamespace(wait_for_stop=wait_for_stop, raise_if_failed=lambda: None)
    robot._wait_for_gripper_stop_bounded("deadman released")

    with pytest.raises(RuntimeError, match="bridge failed closed") as exc_info:
        robot.raise_if_failed()
    assert "HARD STOP FAILURE" in str(exc_info.value.__cause__)


def test_stop_launches_gripper_stop_before_arm_terminate_and_waits_after() -> None:
    order: list[str] = []
    robot = object.__new__(panda_zmq_server.SafePanda)
    robot._args = SimpleNamespace(stop_timeout_s=0.1)
    robot._state_lock = threading.Lock()
    robot._armed = True
    robot._arming = False
    robot._stopped = False
    robot._command_epoch = 7
    robot._stop_complete = threading.Event()
    robot._arm_worker = SimpleNamespace(stop=lambda: order.append("arm-latched"))
    robot._gripper_worker = SimpleNamespace(
        stop=lambda **_kwargs: (order.append("gripper-stop-launched"), True)[1],
        wait_for_stop=lambda **_kwargs: order.append("gripper-stop-waited"),
    )
    robot._terminate_policy_bounded = lambda _reason: order.append("arm-terminate")

    robot.stop("deadman released")

    assert order == ["arm-latched", "gripper-stop-launched", "arm-terminate", "gripper-stop-waited"]
    assert robot._command_epoch == 8


def test_read_only_stop_does_not_send_gripper_rpc() -> None:
    stop_args: list[bool] = []
    robot = object.__new__(panda_zmq_server.SafePanda)
    robot._args = SimpleNamespace(stop_timeout_s=0.1)
    robot._state_lock = threading.Lock()
    robot._armed = False
    robot._arming = False
    robot._stopped = False
    robot._command_epoch = 0
    robot._stop_complete = threading.Event()
    robot._arm_worker = SimpleNamespace(stop=lambda: None)
    robot._gripper_worker = SimpleNamespace(
        stop=lambda *, ensure_hardware_stop: (stop_args.append(ensure_hardware_stop), False)[1],
        wait_for_stop=lambda **_kwargs: pytest.fail("read-only stop must not wait for a Hand Stop RPC"),
    )
    robot._terminate_policy_bounded = lambda _reason: pytest.fail("read-only stop must not terminate a policy")

    robot.stop("shadow shutdown")

    assert stop_args == [False]


def test_arm_timeout_keeps_non_daemon_guardian_until_late_terminate(monkeypatch: pytest.MonkeyPatch) -> None:
    start_entered = threading.Event()
    release_start = threading.Event()
    terminate_reasons: list[str] = []

    class FakeRobot:
        def start_joint_impedance(self) -> None:
            start_entered.set()
            release_start.wait(1.0)

    robot = object.__new__(panda_zmq_server.SafePanda)
    robot._robot = FakeRobot()
    robot._args = SimpleNamespace(arm_timeout_s=0.01)
    robot._state_lock = threading.Lock()
    robot._armed = False
    robot._arming = False
    robot._stopped = False
    robot._command_epoch = 0
    robot._armed_at = None
    robot._last_command_time = None
    robot._stop_complete = threading.Event()
    robot._fatal_error = None
    robot._fatal_lock = threading.Lock()
    robot._arm_worker = SimpleNamespace(stop=lambda: None)
    robot._gripper_worker = SimpleNamespace(stop=lambda **_kwargs: False, wait_for_stop=lambda **_kwargs: None)
    robot._terminate_policy_bounded = terminate_reasons.append
    robot._arming_thread = None
    monkeypatch.setattr(builtins, "input", lambda _prompt: "ARM")

    try:
        with pytest.raises(TimeoutError, match="start_joint_impedance timed out"):
            robot.arm(threading.Event())
        assert start_entered.is_set()
        assert robot._arming_thread is not None
        assert robot._arming_thread.is_alive()
        assert not robot._arming_thread.daemon
        assert terminate_reasons == ["arming failed"]

        release_start.set()
        robot._arming_thread.join(timeout=1.0)
        assert not robot._arming_thread.is_alive()
        assert terminate_reasons == ["arming failed", "late arming response"]
    finally:
        release_start.set()
        if robot._arming_thread is not None:
            robot._arming_thread.join(timeout=1.0)


def test_terminate_timeout_keeps_non_daemon_guardian() -> None:
    terminate_entered = threading.Event()
    release_terminate = threading.Event()

    class FakeRobot:
        def terminate_current_policy(self) -> None:
            terminate_entered.set()
            release_terminate.wait(1.0)

    robot = object.__new__(panda_zmq_server.SafePanda)
    robot._robot = FakeRobot()
    robot._args = SimpleNamespace(stop_timeout_s=0.01)
    robot._fatal_error = None
    robot._fatal_lock = threading.Lock()
    robot._gripper_worker = SimpleNamespace(raise_if_failed=lambda: None)
    robot._termination_threads = []
    robot._termination_threads_lock = threading.Lock()

    try:
        robot._terminate_policy_bounded("test timeout")
        assert terminate_entered.is_set()
        assert robot._termination_threads
        assert robot._termination_threads[0].is_alive()
        assert not robot._termination_threads[0].daemon
        with pytest.raises(RuntimeError, match="bridge failed closed"):
            robot.raise_if_failed()
    finally:
        release_terminate.set()
        for thread in robot._termination_threads:
            thread.join(timeout=1.0)
