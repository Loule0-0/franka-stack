from __future__ import annotations

import numpy as np
import pytest

from franka_runtime.chunking import ActionChunkQueue, ChunkQueueConfig
from franka_runtime.contracts import ContractError


def _chunk(*joint_values: float, gripper: float = 0.5) -> np.ndarray:
    safe_base = np.array([0.0, -0.5, 0.0, -1.5, 0.0, 1.0, 0.0])
    return np.asarray(
        [np.concatenate((safe_base + value * 0.001, [gripper])) for value in joint_values], dtype=np.float32
    )


def test_delay_prefix_is_dropped_before_enqueue() -> None:
    queue = ActionChunkQueue(ChunkQueueConfig(max_overlap_steps=0, freshness_ttl_s=10.0))

    update = queue.enqueue(_chunk(0.0, 1.0, 2.0), delay_steps=1, observed_at_s=0.0, now_s=1.0)

    assert update.dropped_delay_steps == 1
    assert update.accepted_steps == 2
    np.testing.assert_array_equal(queue.pop(now_s=1.0), _chunk(1.0)[0])
    np.testing.assert_array_equal(queue.pop(now_s=1.0), _chunk(2.0)[0])


def test_new_chunk_replaces_old_plan_with_bounded_crossfade() -> None:
    queue = ActionChunkQueue(ChunkQueueConfig(max_overlap_steps=2, freshness_ttl_s=10.0))
    queue.enqueue(_chunk(0.0, 0.0, 99.0), delay_steps=0, observed_at_s=0.0, now_s=0.0)

    update = queue.enqueue(_chunk(3.0, 6.0, 9.0, 12.0), delay_steps=0, observed_at_s=1.0, now_s=1.0)

    assert update.blended_overlap_steps == 2
    assert update.accepted_steps == 4
    # For two overlap steps, old weights are 2/3 then 1/3.
    np.testing.assert_allclose(queue.pop(now_s=1.0)[:7], _chunk(1.0)[0, :7], atol=2e-7)
    np.testing.assert_allclose(queue.pop(now_s=1.0)[:7], _chunk(4.0)[0, :7], atol=2e-7)
    # Old tail (99) was replaced rather than retained.
    np.testing.assert_allclose(queue.pop(now_s=1.0)[:7], _chunk(9.0)[0, :7], atol=2e-7)
    np.testing.assert_allclose(queue.pop(now_s=1.0)[:7], _chunk(12.0)[0, :7], atol=2e-7)


def test_overlap_and_capacity_are_hard_bounds() -> None:
    queue = ActionChunkQueue(ChunkQueueConfig(max_queue_size=3, max_overlap_steps=1, freshness_ttl_s=10.0))
    queue.enqueue(_chunk(0.0, 0.0), delay_steps=0, observed_at_s=0.0, now_s=0.0)

    update = queue.enqueue(_chunk(1.0, 2.0, 3.0, 4.0), delay_steps=0, observed_at_s=1.0, now_s=1.0)

    assert update.blended_overlap_steps == 1
    assert update.accepted_steps == 3
    assert update.dropped_capacity_steps == 1
    assert queue.qsize(now_s=1.0) == 3


def test_empty_queue_never_reuses_last_action() -> None:
    queue = ActionChunkQueue(ChunkQueueConfig(freshness_ttl_s=10.0))
    queue.enqueue(_chunk(0.5), delay_steps=0, observed_at_s=0.0, now_s=0.0)

    assert queue.pop(now_s=0.0) is not None
    assert queue.pop(now_s=0.0) is None
    assert queue.pop(now_s=0.0) is None


def test_stale_incoming_chunk_is_rejected_without_destroying_fresh_queue() -> None:
    queue = ActionChunkQueue(ChunkQueueConfig(max_overlap_steps=0, freshness_ttl_s=2.0))
    queue.enqueue(_chunk(1.0), delay_steps=0, observed_at_s=9.0, now_s=10.0)

    update = queue.enqueue(_chunk(2.0), delay_steps=0, observed_at_s=0.0, now_s=10.0)

    assert update.rejected_stale
    assert update.accepted_steps == 0
    np.testing.assert_array_equal(queue.pop(now_s=10.0), _chunk(1.0)[0])


def test_queued_actions_expire_by_observation_freshness_ttl() -> None:
    queue = ActionChunkQueue(ChunkQueueConfig(freshness_ttl_s=0.5))
    queue.enqueue(_chunk(1.0, 2.0), delay_steps=0, observed_at_s=1.0, now_s=1.1)

    assert queue.qsize(now_s=1.4) == 2
    assert queue.pop(now_s=1.5) is None
    assert queue.qsize(now_s=1.5) == 0


def test_all_delay_dropped_chunk_leaves_existing_fresh_plan() -> None:
    queue = ActionChunkQueue(ChunkQueueConfig(max_overlap_steps=0, freshness_ttl_s=10.0))
    queue.enqueue(_chunk(1.0), delay_steps=0, observed_at_s=0.0, now_s=0.0)

    update = queue.enqueue(_chunk(2.0, 3.0), delay_steps=5, observed_at_s=1.0, now_s=1.0)

    assert update.accepted_steps == 0
    assert update.dropped_delay_steps == 2
    np.testing.assert_array_equal(queue.pop(now_s=1.0), _chunk(1.0)[0])


def test_queue_rejects_nonfinite_or_semantically_invalid_chunks() -> None:
    queue = ActionChunkQueue()
    nonfinite = _chunk(1.0)
    nonfinite[0, 0] = np.nan

    with pytest.raises(ContractError, match="NaN or infinity"):
        queue.enqueue(nonfinite, delay_steps=0, observed_at_s=0.0, now_s=0.0)
    with pytest.raises(ContractError, match="gripper_closed_fraction"):
        queue.enqueue(_chunk(1.0, gripper=1.1), delay_steps=0, observed_at_s=0.0, now_s=0.0)


def test_enqueue_copies_caller_owned_chunk() -> None:
    queue = ActionChunkQueue(ChunkQueueConfig(max_overlap_steps=0, freshness_ttl_s=10.0))
    chunk = _chunk(1.0)
    queue.enqueue(chunk, delay_steps=0, observed_at_s=0.0, now_s=0.0)
    chunk[:] = 7.0

    np.testing.assert_array_equal(queue.pop(now_s=0.0), _chunk(1.0)[0])
