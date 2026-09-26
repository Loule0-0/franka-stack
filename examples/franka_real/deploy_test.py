from __future__ import annotations

# The unit under test deliberately keeps its hardware-independent helpers
# private; direct access here prevents tests from widening the runtime API.
# ruff: noqa: SLF001
import argparse
import builtins
import queue
import sys
import threading
import time

from franka_runtime import EXPECTED_POLICY_METADATA
import numpy as np
import pytest

from examples.franka_real import deploy

SAFE_ACTION = np.array([0.0, -0.5, 0.0, -1.5, 0.0, 1.0, 0.0, 0.5], dtype=np.float32)


def _safe_chunk() -> np.ndarray:
    return np.tile(SAFE_ACTION, (EXPECTED_POLICY_METADATA.action_horizon, 1))


class _FakePolicy:
    def __init__(self, actions: np.ndarray) -> None:
        self.actions = actions
        self.calls = 0
        self.closed = False

    def infer(self, _obs: dict) -> dict:
        self.calls += 1
        return {"actions": self.actions}

    def get_server_metadata(self) -> dict:
        return EXPECTED_POLICY_METADATA.to_mapping()

    def close(self) -> None:
        self.closed = True


def test_observation_preserves_rgb_channel_order() -> None:
    exterior = np.zeros((2, 3, 3), dtype=np.uint8)
    exterior[0, 0] = [1, 2, 3]
    wrist = np.full((2, 3, 3), 7, dtype=np.uint8)
    observation = deploy._build_policy_observation(
        exterior_rgb=exterior,
        wrist_rgb=wrist,
        state=SAFE_ACTION,
        prompt="  pick up the block  ",
    )

    np.testing.assert_array_equal(observation["exterior_image"][0, 0], [1, 2, 3])
    assert observation["prompt"] == "pick up the block"


def test_policy_response_requires_exact_horizon_and_action_dim() -> None:
    good = _FakePolicy(_safe_chunk())
    chunk = deploy._request_action_chunk(good, {}, expected_horizon=EXPECTED_POLICY_METADATA.action_horizon)
    assert chunk.shape == (EXPECTED_POLICY_METADATA.action_horizon, 8)

    bad = _FakePolicy(np.zeros((EXPECTED_POLICY_METADATA.action_horizon, 9), dtype=np.float32))
    with pytest.raises(ValueError, match="action chunk"):
        deploy._request_action_chunk(bad, {}, expected_horizon=EXPECTED_POLICY_METADATA.action_horizon)


def test_metadata_handshake_is_exact() -> None:
    assert deploy._require_expected_metadata(EXPECTED_POLICY_METADATA.to_mapping()) == EXPECTED_POLICY_METADATA
    wrong = EXPECTED_POLICY_METADATA.to_mapping()
    wrong["control_hz"] = 10.0
    with pytest.raises(RuntimeError, match="does not exactly match"):
        deploy._require_expected_metadata(wrong)


def test_warmup_is_discarded_before_realtime_reconnect() -> None:
    actions = _safe_chunk()
    warmup = _FakePolicy(actions)
    realtime = _FakePolicy(actions)
    runtime = object.__new__(deploy.FrankaDeployment)
    runtime._warmup_policy = warmup
    runtime._policy = None
    runtime._metadata = EXPECTED_POLICY_METADATA
    runtime._args = argparse.Namespace(inference_timeout_s=5.0)
    runtime._stop = threading.Event()
    runtime._robot = argparse.Namespace(stop=lambda _reason: None)
    runtime._capture = lambda: ({}, 0.0)
    runtime._connect_policy = lambda _timeout: realtime

    runtime._prepare_policy_sessions()

    assert warmup.calls == 1
    assert warmup.closed
    assert runtime._warmup_policy is None
    assert runtime._policy is realtime
    assert realtime.calls == 0


def test_validate_args_requires_explicit_distinct_camera_roles() -> None:
    args = argparse.Namespace(
        prompt="test",
        policy_host="127.0.0.1",
        policy_port=8000,
        policy_connect_timeout_s=1.0,
        warmup_timeout_s=2.0,
        inference_timeout_s=1.0,
        allow_remote_policy=False,
        polymetis_host="127.0.0.1",
        arm_port=50051,
        gripper_port=50052,
        allow_remote_polymetis=False,
        state_poll_hz=50.0,
        state_timeout_s=0.5,
        command_watchdog_timeout_s=0.25,
        control_loop_watchdog_timeout_s=0.25,
        arm_timeout_s=5.0,
        stop_timeout_s=1.0,
        camera_host="127.0.0.1",
        exterior_camera_port=5000,
        wrist_camera_port=5000,
        camera_timeout_ms=100,
        allow_remote_camera_pickle=False,
        duration_s=0.0,
        queue_refill_threshold=2,
        max_queue_size=10,
        max_overlap_steps=2,
        action_ttl_s=1.0,
        watchdog_timeout_s=2.0,
        max_joint_step_rad=0.01,
        max_joint_acceleration_rad_s2=1.0,
        gripper_deadband=0.05,
        gripper_min_interval_s=0.1,
        gripper_speed=0.05,
        gripper_force=20.0,
    )

    with pytest.raises(ValueError, match="must be different"):
        deploy._validate_args(args)

    args.wrist_camera_port = 5001
    deploy._validate_args(args)
    args.polymetis_host = "192.168.10.2"
    with pytest.raises(ValueError, match="Polymetis must use loopback"):
        deploy._validate_args(args)
    args.allow_remote_polymetis = True
    deploy._validate_args(args)


def test_state_reader_times_out_instead_of_blocking_on_wedged_rpc() -> None:
    release = threading.Event()

    def blocked_read() -> np.ndarray:
        release.wait(1.0)
        return np.zeros(8, dtype=np.float32)

    reader = deploy._LatestStateReader(blocked_read, poll_hz=20.0)
    try:
        with pytest.raises(TimeoutError, match="first Polymetis state"):
            reader.read(timeout_s=0.02)
    finally:
        release.set()
        reader.close(join_timeout_s=0.2)


def test_independent_control_watchdog_stops_robot_without_main_loop() -> None:
    stopped: list[str] = []
    runtime = object.__new__(deploy.FrankaDeployment)
    runtime._stop = threading.Event()
    runtime._errors = queue.Queue()
    runtime._heartbeat_lock = threading.Lock()
    runtime._last_control_heartbeat_s = time.monotonic() - 1.0
    runtime._args = argparse.Namespace(control_loop_watchdog_timeout_s=0.02)
    runtime._robot = argparse.Namespace(armed=True, stop=stopped.append)

    runtime._control_watchdog_loop()

    assert runtime._stop.is_set()
    assert stopped
    assert stopped[0].startswith("control-loop heartbeat expired")
    assert isinstance(runtime._errors.get_nowait(), TimeoutError)


def test_arm_worker_reterminates_after_update_completes_late(monkeypatch: pytest.MonkeyPatch) -> None:
    update_started = threading.Event()
    release_update = threading.Event()
    late_terminate = threading.Event()

    class FakeRobot:
        def update_desired_joint_positions(self, _target: object) -> None:
            update_started.set()
            release_update.wait(1.0)

    monkeypatch.setitem(
        sys.modules, "torch", argparse.Namespace(as_tensor=lambda value, **_kwargs: value, float32=object())
    )
    worker = deploy._LatestArmWorker(FakeRobot(), lambda _reason: late_terminate.set())
    try:
        assert not worker._thread.daemon
        worker.submit(np.zeros(7, dtype=np.float32))
        assert update_started.wait(1.0)
        worker.stop()
        release_update.set()
        assert late_terminate.wait(1.0)
    finally:
        release_update.set()
        worker.close()


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

    worker = deploy._LatestGripperWorker(FakeGripper())
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

    worker = deploy._LatestGripperWorker(FakeGripper())
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

    worker = deploy._LatestGripperWorker(FakeGripper())
    command_thread = worker._thread
    try:
        assert worker.stop(ensure_hardware_stop=True)
        worker._thread = MustNotJoin()
        worker.wait_for_stop(timeout_s=0.01)
    finally:
        worker._thread = command_thread
        worker.close(join_timeout_s=1.0)


def test_gripper_stop_timeout_becomes_hard_stop_failure() -> None:
    panda = object.__new__(deploy.PolymetisPanda)
    panda._args = argparse.Namespace(stop_timeout_s=0.01)
    panda._state_lock = threading.Lock()
    panda._termination_failure = None

    def wait_for_stop(*, timeout_s: float) -> None:
        assert timeout_s == 0.01
        raise TimeoutError("gripper stop RPC timed out")

    panda._gripper_worker = argparse.Namespace(wait_for_stop=wait_for_stop)
    panda._wait_for_gripper_stop_bounded("deadman released")

    with pytest.raises(RuntimeError, match="HARD STOP FAILURE.*gripper stop RPC timed out"):
        panda.raise_if_failed()


def test_stop_launches_gripper_stop_before_arm_terminate_and_waits_after() -> None:
    order: list[str] = []
    panda = object.__new__(deploy.PolymetisPanda)
    panda._args = argparse.Namespace(stop_timeout_s=0.1)
    panda._state_lock = threading.Lock()
    panda._armed = True
    panda._arming = False
    panda._stopped = False
    panda._stop_complete = threading.Event()
    panda._termination_failure = None
    panda._arm_worker = argparse.Namespace(stop=lambda: order.append("arm-latched"))
    panda._gripper_worker = argparse.Namespace(
        stop=lambda **_kwargs: (order.append("gripper-stop-launched"), True)[1],
        wait_for_stop=lambda **_kwargs: order.append("gripper-stop-waited"),
    )
    panda._terminate_policy_bounded = lambda _reason: order.append("arm-terminate")

    panda.stop("deadman released")

    assert order == ["arm-latched", "gripper-stop-launched", "arm-terminate", "gripper-stop-waited"]


def test_read_only_stop_does_not_send_gripper_rpc() -> None:
    stop_args: list[bool] = []
    panda = object.__new__(deploy.PolymetisPanda)
    panda._args = argparse.Namespace(stop_timeout_s=0.1)
    panda._state_lock = threading.Lock()
    panda._armed = False
    panda._arming = False
    panda._stopped = False
    panda._stop_complete = threading.Event()
    panda._arm_worker = argparse.Namespace(stop=lambda: None)
    panda._gripper_worker = argparse.Namespace(
        stop=lambda *, ensure_hardware_stop: (stop_args.append(ensure_hardware_stop), False)[1],
        wait_for_stop=lambda **_kwargs: pytest.fail("read-only stop must not wait for a Hand Stop RPC"),
    )
    panda._terminate_policy_bounded = lambda _reason: pytest.fail("read-only stop must not terminate a policy")

    panda.stop("shadow shutdown")

    assert stop_args == [False]


def test_terminate_timeout_is_a_persistent_hard_failure() -> None:
    release = threading.Event()

    class FakeRobot:
        def terminate_current_policy(self) -> None:
            release.wait(1.0)

    panda = object.__new__(deploy.PolymetisPanda)
    panda._robot = FakeRobot()
    panda._args = argparse.Namespace(stop_timeout_s=0.01)
    panda._state_lock = threading.Lock()
    panda._termination_failure = None
    panda._termination_threads = []
    panda._termination_threads_lock = threading.Lock()

    try:
        panda._terminate_policy_bounded("test timeout")
        assert panda._termination_threads
        assert not panda._termination_threads[0].daemon
        with pytest.raises(RuntimeError, match="HARD STOP FAILURE"):
            panda.raise_if_failed()
    finally:
        release.set()
        for thread in panda._termination_threads:
            thread.join(timeout=1.0)


def test_arm_timeout_keeps_non_daemon_guardian_until_late_terminate(monkeypatch: pytest.MonkeyPatch) -> None:
    start_entered = threading.Event()
    release_start = threading.Event()
    terminate_reasons: list[str] = []

    class FakeRobot:
        def start_joint_impedance(self) -> None:
            start_entered.set()
            release_start.wait(1.0)

    panda = object.__new__(deploy.PolymetisPanda)
    panda._robot = FakeRobot()
    panda._args = argparse.Namespace(arm_timeout_s=0.01)
    panda._state_lock = threading.Lock()
    panda._armed = False
    panda._arming = False
    panda._stopped = False
    panda._termination_failure = None
    panda._stop_complete = threading.Event()
    panda._arm_worker = argparse.Namespace(stop=lambda: None, reset_freshness=lambda **_kwargs: None)
    panda._gripper_worker = argparse.Namespace(stop=lambda **_kwargs: False, wait_for_stop=lambda **_kwargs: None)
    panda._terminate_policy_bounded = terminate_reasons.append
    panda._arming_thread = None
    monkeypatch.setattr(builtins, "input", lambda _prompt: "ARM")

    try:
        with pytest.raises(TimeoutError, match="start_joint_impedance timed out"):
            panda.arm(threading.Event())
        assert start_entered.is_set()
        assert panda._arming_thread is not None
        assert panda._arming_thread.is_alive()
        assert not panda._arming_thread.daemon
        assert terminate_reasons == ["arming failed"]

        release_start.set()
        panda._arming_thread.join(timeout=1.0)
        assert not panda._arming_thread.is_alive()
        assert terminate_reasons == ["arming failed", "late arming response"]
    finally:
        release_start.set()
        if panda._arming_thread is not None:
            panda._arming_thread.join(timeout=1.0)
