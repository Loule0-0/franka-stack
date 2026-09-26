#!/usr/bin/env python3
"""Fail a Franka LeRobot v2.1 dataset audit on contract or value errors."""

from __future__ import annotations

import argparse
from collections.abc import Iterable
import json
from pathlib import Path

from franka_runtime.dataset_provenance import FRANKA_DATASET_MANIFEST_KEYS
from franka_runtime.dataset_provenance import DatasetContentDigest
from franka_runtime.dataset_provenance import verify_dataset_content_digest
import numpy as np

SCHEMA_ID = "franka-runtime/v1"
EXPECTED_FPS = 20
EXPECTED_PERIOD_S = 1.0 / EXPECTED_FPS
MAX_FRAME_GAP_S = 0.125
MAX_MEDIAN_PERIOD_ERROR_S = 0.01
MAX_PERIOD_JITTER_S = 0.03
MIN_CADENCE_RATIO = 0.9
EXPECTED_COLOR_SPACE = "rgb_uint8_hwc"
EXPECTED_STATE = "q7_rad + gripper_closed"
EXPECTED_ACTION = "absolute_q7_rad + gripper_closed"
EXPECTED_CAMERA_ROLES = {"exterior": "base_rgb", "wrist": "wrist_rgb"}
EXPECTED_MANIFEST_KEYS = FRANKA_DATASET_MANIFEST_KEYS
EXPECTED_EPISODE_KEYS = {
    "episode",
    "outcome",
    "raw_frames",
    "frames",
    "duration_s",
    "max_frame_gap_s",
    "median_period_s",
    "p95_period_jitter_s",
    "cadence_ratio",
}
ACCEPTED_OUTCOMES = frozenset({"complete", "deadman_released", "operator_stopped"})
EXPECTED_FEATURES = {
    "observation.images.exterior": ("image", None),
    "observation.images.wrist": ("image", None),
    "observation.state": ("float32", (8,)),
    "action": ("float32", (8,)),
}
JOINT_LOW = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
JOINT_HIGH = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
JOINT_MARGIN_RAD = 0.02
MAX_JOINT_STEP_RAD = 0.025
MAX_JOINT_ACCELERATION_RAD_S2 = 3.0
SAFETY_STEP_ATOL_RAD = 1e-6
SAFETY_ACCELERATION_ATOL_RAD_S2 = 1e-3


def _as_numpy(value: object) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    return np.asarray(value)


def validate_feature_contract(features: dict) -> None:
    missing = sorted(set(EXPECTED_FEATURES) - set(features))
    if missing:
        raise ValueError(f"dataset is missing required features: {missing}")
    for key, (expected_dtype, expected_shape) in EXPECTED_FEATURES.items():
        feature = features[key]
        if feature.get("dtype") != expected_dtype:
            raise ValueError(f"{key} dtype must be {expected_dtype!r}, got {feature.get('dtype')!r}")
        shape = tuple(feature.get("shape", ()))
        if expected_shape is not None and shape != expected_shape:
            raise ValueError(f"{key} shape must be {expected_shape}, got {shape}")
        if expected_dtype == "image" and (len(shape) != 3 or shape[-1] != 3):
            raise ValueError(f"{key} metadata must describe an HWC RGB image, got {shape}")


def validate_manifest_contract(manifest: object, *, repo_id: str) -> list[dict]:
    if not isinstance(manifest, dict):
        raise ValueError("manifest must be a JSON object")
    actual_keys = set(manifest)
    if actual_keys != EXPECTED_MANIFEST_KEYS:
        missing = sorted(EXPECTED_MANIFEST_KEYS - actual_keys)
        extra = sorted(actual_keys - EXPECTED_MANIFEST_KEYS)
        raise ValueError(f"manifest fields must exactly match the contract; missing={missing}, extra={extra}")

    expected_values = {
        "schema_id": SCHEMA_ID,
        "repo_id": repo_id,
        "fps": EXPECTED_FPS,
        "color_space": EXPECTED_COLOR_SPACE,
        "state": EXPECTED_STATE,
        "action": EXPECTED_ACTION,
        "camera_roles": EXPECTED_CAMERA_ROLES,
    }
    for key, expected in expected_values.items():
        actual = manifest[key]
        if actual != expected or (key == "fps" and type(actual) is not int):
            raise ValueError(f"manifest {key} must be exactly {expected!r}, got {actual!r}")
    if not isinstance(manifest["task"], str) or not manifest["task"].strip():
        raise ValueError("manifest task must be a non-empty string")
    DatasetContentDigest.from_mapping(manifest["content_digest"])

    episodes = manifest["episodes"]
    if not isinstance(episodes, list) or not episodes:
        raise ValueError("manifest episodes must be a non-empty list")
    episode_names: set[str] = set()
    for index, summary in enumerate(episodes):
        if not isinstance(summary, dict) or set(summary) != EXPECTED_EPISODE_KEYS:
            actual = sorted(summary) if isinstance(summary, dict) else type(summary).__name__
            raise ValueError(f"manifest episode {index} fields must exactly match the contract, got {actual}")
        name = summary["episode"]
        if not isinstance(name, str) or not name.strip() or name in episode_names:
            raise ValueError(f"manifest episode {index} must have a unique non-empty name")
        episode_names.add(name)
        if summary["outcome"] not in ACCEPTED_OUTCOMES:
            raise ValueError(f"manifest episode {index} has invalid outcome {summary['outcome']!r}")
        for key in ("raw_frames", "frames"):
            value = summary[key]
            if type(value) is not int or value < 2:
                raise ValueError(f"manifest episode {index} {key} must be an integer >= 2")
        if summary["raw_frames"] != summary["frames"]:
            raise ValueError(f"manifest episode {index} raw_frames and frames must match")
        for key in ("duration_s", "max_frame_gap_s", "median_period_s", "p95_period_jitter_s", "cadence_ratio"):
            value = summary[key]
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))  # noqa: UP038 -- Kept importable under Python 3.8.
                or not np.isfinite(value)
            ):
                raise ValueError(f"manifest episode {index} {key} must be a finite number")
        if summary["duration_s"] <= 0.0:
            raise ValueError(f"manifest episode {index} duration_s must be positive")
        if (
            summary["max_frame_gap_s"] < 0.0
            or summary["median_period_s"] <= 0.0
            or summary["p95_period_jitter_s"] < 0.0
        ):
            raise ValueError(f"manifest episode {index} cadence measurements must be non-negative")
        if not 0.0 < summary["cadence_ratio"] <= 1.0:
            raise ValueError(f"manifest episode {index} cadence_ratio must be in (0, 1]")
        if summary["max_frame_gap_s"] > MAX_FRAME_GAP_S:
            raise ValueError(f"manifest episode {index} max_frame_gap_s exceeds {MAX_FRAME_GAP_S}")
        if abs(summary["median_period_s"] - EXPECTED_PERIOD_S) > MAX_MEDIAN_PERIOD_ERROR_S:
            raise ValueError(f"manifest episode {index} median_period_s is not consistent with {EXPECTED_FPS} Hz")
        if summary["p95_period_jitter_s"] > MAX_PERIOD_JITTER_S:
            raise ValueError(f"manifest episode {index} p95_period_jitter_s exceeds {MAX_PERIOD_JITTER_S}")
        if summary["cadence_ratio"] < MIN_CADENCE_RATIO:
            raise ValueError(f"manifest episode {index} cadence_ratio is below {MIN_CADENCE_RATIO}")
    return episodes


def validate_dataset_counts(
    episodes: list[dict],
    *,
    num_episodes: int,
    num_frames: int,
    metadata_episode_lengths: list[int],
) -> None:
    if len(episodes) != num_episodes:
        raise ValueError(f"manifest has {len(episodes)} episodes but dataset metadata reports {num_episodes}")
    manifest_frames = sum(summary["frames"] for summary in episodes)
    if manifest_frames != num_frames:
        raise ValueError(f"manifest has {manifest_frames} frames but dataset metadata reports {num_frames}")
    if len(metadata_episode_lengths) != num_episodes:
        raise ValueError(
            "dataset metadata episode table has "
            f"{len(metadata_episode_lengths)} entries but reports {num_episodes} episodes"
        )
    metadata_frames = sum(metadata_episode_lengths)
    if metadata_frames != num_frames:
        raise ValueError(
            f"dataset metadata episode lengths total {metadata_frames} but dataset reports {num_frames} frames"
        )


def validate_dataset_tasks(tasks: Iterable[object], *, expected_task: str) -> None:
    normalized: set[str] = set()
    for task in tasks:
        if not isinstance(task, str) or not task.strip():
            raise ValueError("dataset task metadata must contain only non-empty strings")
        normalized.add(task.strip())
    if normalized != {expected_task.strip()}:
        raise ValueError(f"dataset must contain exactly manifest task {expected_task!r}, got {sorted(normalized)!r}")


def validate_sample(sample: dict, *, index: int) -> None:
    state = _as_numpy(sample["observation.state"])
    if state.shape != (8,) or not np.isfinite(state).all():
        raise ValueError(f"frame {index}: observation.state must be finite shape (8,), got {state.shape}")
    if np.any(state[:7] < JOINT_LOW) or np.any(state[:7] > JOINT_HIGH):
        raise ValueError(f"frame {index}: observation.state is outside Panda physical joint limits")
    if not 0.0 <= float(state[7]) <= 1.0:
        raise ValueError(f"frame {index}: observation.state[7] must be gripper_closed_fraction in [0, 1]")

    action = _as_numpy(sample["action"])
    if action.shape != (8,) or not np.isfinite(action).all():
        raise ValueError(f"frame {index}: action must be finite shape (8,), got {action.shape}")
    if np.any(action[:7] < JOINT_LOW + JOINT_MARGIN_RAD) or np.any(action[:7] > JOINT_HIGH - JOINT_MARGIN_RAD):
        raise ValueError(f"frame {index}: action is outside Panda joint limits with {JOINT_MARGIN_RAD:.2f} rad margin")
    if not 0.0 <= float(action[7]) <= 1.0:
        raise ValueError(f"frame {index}: action[7] must be gripper_closed_fraction in [0, 1]")

    for key in ("observation.images.exterior", "observation.images.wrist"):
        image = _as_numpy(sample[key])
        if image.ndim != 3 or 3 not in (image.shape[0], image.shape[-1]):
            raise ValueError(f"frame {index}: {key} must be CHW or HWC RGB, got {image.shape}")
        if image.dtype.kind not in "uif" or not np.isfinite(image).all():
            raise ValueError(f"frame {index}: {key} must contain finite numeric RGB values")
        if image.dtype.kind == "f" and (image.min() < 0.0 or image.max() > 1.0):
            raise ValueError(f"frame {index}: floating-point {key} must be in [0, 1]")
        if image.dtype.kind in "ui" and (image.min() < 0 or image.max() > 255):
            raise ValueError(f"frame {index}: integer {key} must be in [0, 255]")

    task = sample.get("task")
    if not isinstance(task, str) or not task.strip():
        raise ValueError(f"frame {index}: task prompt must be a non-empty string")


def validate_command_safety_sequence(samples: Iterable[dict], *, episode_lengths: list[int]) -> None:
    """Recheck the recorder's stateful command limits across every episode."""

    sample_iterator = iter(samples)
    global_index = 0
    period_s = 1.0 / EXPECTED_FPS
    for episode_index, episode_length in enumerate(episode_lengths):
        previous_velocity = np.zeros(7, dtype=np.float64)
        for frame_index in range(episode_length):
            try:
                sample = next(sample_iterator)
            except StopIteration as exc:
                raise ValueError(
                    f"command sequence ended inside episode {episode_index} at frame {frame_index}"
                ) from exc
            state = _as_numpy(sample["observation.state"])
            action = _as_numpy(sample["action"])
            if (
                state.shape != (8,)
                or action.shape != (8,)
                or not np.isfinite(state).all()
                or not np.isfinite(action).all()
            ):
                raise ValueError(f"frame {global_index}: cannot audit command safety without finite 8D state/action")
            command_step = np.asarray(action[:7] - state[:7], dtype=np.float64)
            max_step = float(np.max(np.abs(command_step)))
            if max_step > MAX_JOINT_STEP_RAD + SAFETY_STEP_ATOL_RAD:
                raise ValueError(
                    f"frame {global_index}: commanded joint step {max_step:.6f} rad "
                    f"exceeds {MAX_JOINT_STEP_RAD:.6f} rad"
                )
            commanded_velocity = command_step / period_s
            max_acceleration = float(np.max(np.abs(commanded_velocity - previous_velocity)) / period_s)
            if max_acceleration > MAX_JOINT_ACCELERATION_RAD_S2 + SAFETY_ACCELERATION_ATOL_RAD_S2:
                raise ValueError(
                    f"frame {global_index}: commanded joint acceleration {max_acceleration:.6f} rad/s^2 "
                    f"exceeds {MAX_JOINT_ACCELERATION_RAD_S2:.6f} rad/s^2"
                )
            previous_velocity = commanded_velocity
            global_index += 1
    try:
        next(sample_iterator)
    except StopIteration:
        return
    raise ValueError("command sequence contains frames beyond the declared episode lengths")


def sample_indices(total: int, maximum: int | None) -> np.ndarray:
    if total < 1:
        raise ValueError("dataset contains no frames")
    if maximum is None or maximum >= total:
        return np.arange(total, dtype=np.int64)
    if maximum < 1:
        raise ValueError("--max-frames must be positive")
    return np.unique(np.linspace(0, total - 1, num=maximum, dtype=np.int64))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, required=True, help="Exact LeRobot repository directory")
    parser.add_argument("--repo-id", default="local/franka_gello")
    parser.add_argument("--max-frames", type=int, default=2_000, help="Use 0 to audit every frame")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    root = args.dataset_root.expanduser().resolve()
    manifest_path = root / "franka_manifest.json"
    if not manifest_path.is_file():
        raise FileNotFoundError(f"missing converter manifest: {manifest_path}")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    episodes = validate_manifest_contract(manifest, repo_id=args.repo_id)
    verify_dataset_content_digest(root, manifest["content_digest"])

    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

    dataset = LeRobotDataset(args.repo_id, root=root, download_videos=False)
    if dataset.meta.robot_type != "franka_panda":
        raise ValueError(f"robot_type must be 'franka_panda', got {dataset.meta.robot_type!r}")
    if dataset.fps != EXPECTED_FPS:
        raise ValueError(f"dataset fps must be {EXPECTED_FPS}, got {dataset.fps}")
    if dataset.num_episodes < 1:
        raise ValueError("dataset contains no episodes")
    metadata_episode_lengths = [int(episode["length"]) for episode in dataset.meta.episodes.values()]
    if any(length < 2 for length in metadata_episode_lengths):
        raise ValueError("every episode must contain at least two frames")
    validate_dataset_counts(
        episodes,
        num_episodes=dataset.num_episodes,
        num_frames=dataset.num_frames,
        metadata_episode_lengths=metadata_episode_lengths,
    )
    validate_command_safety_sequence(
        (dataset.hf_dataset[index] for index in range(dataset.num_frames)),
        episode_lengths=metadata_episode_lengths,
    )
    validate_dataset_tasks(dataset.meta.tasks.values(), expected_task=manifest["task"])
    validate_feature_contract(dataset.features)

    maximum = None if args.max_frames == 0 else args.max_frames
    indices = sample_indices(dataset.num_frames, maximum)
    for index in indices:
        sample = dataset[int(index)]
        validate_sample(sample, index=int(index))
        if sample["task"].strip() != manifest["task"].strip():
            raise ValueError(f"frame {int(index)}: task does not match the manifest task")

    print(
        f"[audit passed] root={root} episodes={dataset.num_episodes} "
        f"frames={dataset.num_frames} audited={len(indices)} fps={dataset.fps}"
    )


if __name__ == "__main__":
    main()
