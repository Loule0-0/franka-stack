"""Thread-safe action chunk scheduling with no robot or networking dependencies."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import threading
import time
from typing import Callable

import numpy as np
import numpy.typing as npt

from franka_runtime.contracts import validate_action_chunk


def _plain_nonnegative_int(name: str, value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


def _finite_time(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float, np.integer, np.floating)):
        raise ValueError(f"{name} must be a number")
    result = float(value)
    if not np.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


@dataclass(frozen=True)
class ChunkQueueConfig:
    max_queue_size: int = 32
    max_overlap_steps: int = 4
    freshness_ttl_s: float = 1.0

    def __post_init__(self) -> None:
        max_queue_size = _plain_nonnegative_int("max_queue_size", self.max_queue_size)
        if max_queue_size < 1:
            raise ValueError("max_queue_size must be >= 1")
        _plain_nonnegative_int("max_overlap_steps", self.max_overlap_steps)
        ttl = _finite_time("freshness_ttl_s", self.freshness_ttl_s)
        if ttl <= 0.0:
            raise ValueError("freshness_ttl_s must be > 0")


@dataclass(frozen=True)
class ChunkUpdate:
    """Result of one asynchronous inference result being offered to the queue."""

    accepted_steps: int
    dropped_delay_steps: int
    blended_overlap_steps: int
    dropped_capacity_steps: int
    rejected_stale: bool


@dataclass
class _QueuedAction:
    value: npt.NDArray[np.float32]
    expires_at_s: float


class ActionChunkQueue:
    """A deterministic producer/consumer queue for asynchronous policy chunks.

    ``enqueue`` aligns a newly returned chunk by dropping its delayed prefix,
    cross-fades only a bounded prefix with the current plan, and replaces the
    remainder of that old plan.  ``pop`` returns ``None`` when empty; the queue
    deliberately keeps no "last action" fallback.

    All time values must share the clock domain used by ``clock`` (monotonic
    time by default).  Explicit ``now_s`` values make the logic easy to test.
    """

    def __init__(
        self,
        config: ChunkQueueConfig | None = None,
        *,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.config = config or ChunkQueueConfig()
        self._clock = clock
        self._queue: deque[_QueuedAction] = deque()
        self._lock = threading.Lock()

    def _now(self, now_s: float | None) -> float:
        return _finite_time("now_s", self._clock() if now_s is None else now_s)

    def _purge_expired_locked(self, now_s: float) -> None:
        if not self._queue:
            return
        self._queue = deque(item for item in self._queue if item.expires_at_s > now_s)

    def qsize(self, *, now_s: float | None = None) -> int:
        now = self._now(now_s)
        with self._lock:
            self._purge_expired_locked(now)
            return len(self._queue)

    def clear(self) -> None:
        with self._lock:
            self._queue.clear()

    def pop(self, *, now_s: float | None = None) -> npt.NDArray[np.float32] | None:
        """Pop one fresh action, or ``None`` without replaying an old action."""

        now = self._now(now_s)
        with self._lock:
            self._purge_expired_locked(now)
            if not self._queue:
                return None
            return self._queue.popleft().value.copy()

    def enqueue(
        self,
        chunk: object,
        *,
        delay_steps: int,
        observed_at_s: float,
        now_s: float | None = None,
    ) -> ChunkUpdate:
        """Offer one completed inference chunk to the execution queue.

        ``observed_at_s`` is when the policy input was captured, not when the
        inference response arrived.  A result older than ``freshness_ttl_s`` is
        rejected in full.  ``delay_steps`` accounts for actions whose intended
        execution time elapsed while inference was running.
        """

        actions = validate_action_chunk(chunk)
        delay = _plain_nonnegative_int("delay_steps", delay_steps)
        observed_at = _finite_time("observed_at_s", observed_at_s)
        now = self._now(now_s)
        if observed_at > now:
            raise ValueError("observed_at_s cannot be later than now_s")

        dropped_delay = min(delay, len(actions))
        fresh = actions[dropped_delay:].copy()

        with self._lock:
            self._purge_expired_locked(now)
            if now - observed_at >= self.config.freshness_ttl_s:
                return ChunkUpdate(0, dropped_delay, 0, 0, True)
            if len(fresh) == 0:
                return ChunkUpdate(0, dropped_delay, 0, 0, False)

            existing = list(self._queue)
            overlap = min(self.config.max_overlap_steps, len(existing), len(fresh))
            expires_at = observed_at + self.config.freshness_ttl_s
            replacement: list[_QueuedAction] = []

            for index in range(overlap):
                # Linear cross-fade: first action favors the already executing
                # plan; the last overlap action favors the newly inferred plan.
                old_weight = (overlap - index) / (overlap + 1.0)
                blended = old_weight * existing[index].value + (1.0 - old_weight) * fresh[index]
                replacement.append(
                    _QueuedAction(
                        value=np.asarray(blended, dtype=np.float32),
                        expires_at_s=min(existing[index].expires_at_s, expires_at),
                    )
                )

            replacement.extend(_QueuedAction(value=step.copy(), expires_at_s=expires_at) for step in fresh[overlap:])

            dropped_capacity = max(0, len(replacement) - self.config.max_queue_size)
            self._queue = deque(replacement[: self.config.max_queue_size])
            return ChunkUpdate(
                accepted_steps=len(self._queue),
                dropped_delay_steps=dropped_delay,
                blended_overlap_steps=overlap,
                dropped_capacity_steps=dropped_capacity,
                rejected_stale=False,
            )
