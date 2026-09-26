from __future__ import annotations

import signal

import pytest

from franka_runtime.lifecycle import TwoStageShutdown


class HardExitRequested(BaseException):
    pass


def test_first_signal_interrupts_work_and_second_signal_hard_exits() -> None:
    stops: list[None] = []
    exit_codes: list[int] = []

    def hard_exit(code: int) -> None:
        exit_codes.append(code)
        raise HardExitRequested

    shutdown = TwoStageShutdown(lambda: stops.append(None), hard_exit=hard_exit)

    with pytest.raises(KeyboardInterrupt):
        shutdown.handle_signal(signal.SIGTERM, None)
    with pytest.raises(HardExitRequested):
        shutdown.handle_signal(signal.SIGTERM, None)

    assert len(stops) == 1
    assert exit_codes == [128 + int(signal.SIGTERM)]


def test_first_signal_during_cleanup_does_not_interrupt_cleanup() -> None:
    stops: list[None] = []
    exit_codes: list[int] = []

    def hard_exit(code: int) -> None:
        exit_codes.append(code)
        raise HardExitRequested

    shutdown = TwoStageShutdown(lambda: stops.append(None), hard_exit=hard_exit)
    shutdown.begin_cleanup()

    shutdown.handle_signal(signal.SIGINT, None)
    with pytest.raises(HardExitRequested):
        shutdown.handle_signal(signal.SIGINT, None)

    assert len(stops) == 2
    assert exit_codes == [128 + int(signal.SIGINT)]
