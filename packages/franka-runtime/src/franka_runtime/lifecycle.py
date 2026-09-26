"""Process-lifecycle helpers shared by Franka robot-side entry points."""

from __future__ import annotations

import os
from typing import Callable, NoReturn


class TwoStageShutdown:
    """Turn the first signal into cleanup and reserve the second for a hard exit.

    ``begin_cleanup`` must be called before cleanup starts. Once cleanup has
    started, a first signal only repeats the idempotent stop request instead of
    interrupting safety-critical cleanup with another ``KeyboardInterrupt``.
    """

    def __init__(
        self,
        request_stop: Callable[[], None],
        *,
        hard_exit: Callable[[int], NoReturn] = os._exit,
    ) -> None:
        self._request_stop = request_stop
        self._hard_exit = hard_exit
        self._cleanup_started = False
        self._signals_seen = 0

    def begin_cleanup(self) -> None:
        """Mark cleanup active before issuing the idempotent stop request."""

        self._cleanup_started = True
        self._request_stop()

    def handle_signal(self, signum: int, _frame: object) -> None:
        """Request cleanup on the first signal and hard-exit on the second."""

        self._signals_seen += 1
        if self._signals_seen >= 2:
            self._hard_exit(128 + int(signum))
        self._request_stop()
        if not self._cleanup_started:
            raise KeyboardInterrupt
