#!/usr/bin/env python3
"""Convert GELLO pickle episodes into the repository's LeRobot v2.1 contract.

The converter is deliberately strict: camera roles and color space are explicit,
all state/action values must be finite, and an existing dataset is never removed
unless ``--overwrite`` is supplied.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import numbers
from pathlib import Path
import pickle
import re
import shutil

from franka_runtime.dataset_provenance import FRANKA_DATASET_MANIFEST_KEYS
from franka_runtime.dataset_provenance import compute_dataset_content_digest
import numpy as np
from PIL import Image

DEFAULT_REPO_ID = "local/franka_gello"
SCHEMA_ID = "franka-runtime/v1"
TARGET_FPS = 20
MAX_JOINT_STEP_RAD = 0.025
MAX_JOINT_ACCELERATION_RAD_S2 = 3.0
MAX_FRAME_GAP_S = 0.125
MAX_MEDIAN_PERIOD_ERROR_S = 0.01
MAX_PERIOD_JITTER_S = 0.03
MIN_CADENCE_RATIO = 0.9
EXTERIOR_KEY = "base_rgb"
WRIST_KEY = "wrist_rgb"
JOINT_LOW = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
JOINT_HIGH = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
JOINT_MARGIN_RAD = 0.02
ACCEPTED_OUTCOMES = frozenset({"complete", "deadman_released", "operator_stopped"})
REPO_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*/[A-Za-z0-9][A-Za-z0-9._-]*$")
SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")
SAFETY_STEP_ATOL_RAD = 1e-6
SAFETY_ACCELERATION_ATOL_RAD_S2 = 1e-3
RAW_EPISODE_METADATA_KEYS = frozenset(
    {
        "schema_id",
        "task",
        "outcome",
        "recorded_frames",
        "control_hz",
        "max_joint_step_rad",
        "max_joint_acceleration_rad_s2",
        "camera_ports",
        "color_space",
        "gripper_semantics",
        "gello_device_sha256",
        "calibration_sha256",
    }
)


def _resolve_output_path(*, dataset_home: Path, repo_id: str, raw_dir: Path) -> Path:
    """Resolve a canonical two-component repo id without overlapping raw input."""

    if not REPO_ID_PATTERN.fullmatch(repo_id):
        raise ValueError("repo-id must have the canonical namespace/name form")
    dataset_home = dataset_home.expanduser().resolve()
    if dataset_home == Path(dataset_home.anchor):
        raise ValueError("dataset-home must not be a filesystem root")
    candidate = dataset_home
    for component in repo_id.split("/"):
        candidate /= component
        if candidate.is_symlink():
            raise ValueError(f"dataset output path must not traverse a symbolic link: {candidate}")
    output_path = candidate.resolve()
    if dataset_home not in output_path.parents:
        raise ValueError(f"repo-id resolves outside dataset-home: {output_path}")
    raw_dir = raw_dir.expanduser().resolve()
    if output_path == raw_dir or output_path in raw_dir.parents or raw_dir in output_path.parents:
        raise ValueError(f"output dataset must not overlap raw recordings: output={output_path} raw={raw_dir}")
    return output_path


def _validate_owned_overwrite_target(output_path: Path, *, repo_id: str) -> None:
    """Allow recursive replacement only for a dataset this converter owns."""

    if output_path.is_symlink() or not output_path.is_dir():
        raise ValueError(f"overwrite target must be a real dataset directory: {output_path}")
    manifest_path = output_path / "franka_manifest.json"
    if manifest_path.is_symlink() or not manifest_path.is_file():
        raise ValueError(
            f"refusing to overwrite an unowned or incomplete directory without a regular manifest: {output_path}"
        )
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"refusing to overwrite a directory with an invalid manifest: {manifest_path}") from exc
    if not isinstance(manifest, dict) or manifest.get("schema_id") != SCHEMA_ID or manifest.get("repo_id") != repo_id:
        raise ValueError(
            f"refusing to overwrite a directory whose manifest does not exactly identify {SCHEMA_ID!r} / {repo_id!r}"
        )


def _timestamp_from_path(path: Path) -> dt.datetime:
    timestamp = path.stem
    if timestamp.endswith("Z"):
        timestamp = timestamp[:-1] + "+00:00"
    try:
        return dt.datetime.fromisoformat(timestamp)
    except ValueError:
        if "T" not in timestamp:
            raise ValueError(f"frame filename is not an ISO timestamp: {path.name}") from None
        date_part, time_part = timestamp.split("T", 1)
        fields = time_part.split("-", 2)
        if len(fields) != 3:
            raise ValueError(f"frame filename is not an ISO timestamp: {path.name}") from None
        return dt.datetime.fromisoformat(f"{date_part}T{':'.join(fields)}")


def _load_episode(
    path: Path,
    *,
    expected_task: str,
    max_frame_gap_s: float,
    max_period_jitter_s: float,
    min_cadence_ratio: float,
) -> tuple[list[tuple[dt.datetime, dict]], dict, dict[str, float]]:
    metadata_path = path / "episode.json"
    if not metadata_path.is_file():
        raise ValueError(f"{path}: missing episode.json")
    try:
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError) as exc:
        raise ValueError(f"{metadata_path}: invalid episode metadata") from exc
    if not isinstance(metadata, dict):
        raise ValueError(f"{metadata_path}: metadata must be an object")
    actual_metadata_keys = set(metadata)
    if actual_metadata_keys != RAW_EPISODE_METADATA_KEYS:
        missing = sorted(RAW_EPISODE_METADATA_KEYS - actual_metadata_keys)
        unknown = sorted(actual_metadata_keys - RAW_EPISODE_METADATA_KEYS)
        raise ValueError(
            f"{metadata_path}: metadata fields must exactly match the contract; missing={missing}, unknown={unknown}"
        )
    if metadata.get("schema_id") != SCHEMA_ID:
        raise ValueError(f"{metadata_path}: schema_id must be {SCHEMA_ID!r}")
    if metadata.get("outcome") not in ACCEPTED_OUTCOMES:
        raise ValueError(f"{metadata_path}: unacceptable outcome {metadata.get('outcome')!r}")
    control_hz = metadata.get("control_hz")
    if isinstance(control_hz, bool) or not isinstance(control_hz, numbers.Real) or not np.isfinite(control_hz):
        raise ValueError(f"{metadata_path}: control_hz must be the finite number {TARGET_FPS}")
    if not np.isclose(control_hz, TARGET_FPS, rtol=0.0, atol=1e-12):
        raise ValueError(f"{metadata_path}: control_hz must be {TARGET_FPS}")
    for field, expected in (
        ("max_joint_step_rad", MAX_JOINT_STEP_RAD),
        ("max_joint_acceleration_rad_s2", MAX_JOINT_ACCELERATION_RAD_S2),
    ):
        value = metadata.get(field)
        if isinstance(value, bool) or not isinstance(value, numbers.Real) or not np.isfinite(value):
            raise ValueError(f"{metadata_path}: {field} must be the finite number {expected}")
        if not np.isclose(value, expected, rtol=0.0, atol=1e-12):
            raise ValueError(f"{metadata_path}: {field} must be {expected}, got {value}")
    if metadata["color_space"] != "rgb_uint8_hwc":
        raise ValueError(f"{metadata_path}: color_space must be 'rgb_uint8_hwc'")
    if metadata["gripper_semantics"] != "closed_fraction_0_open_1_closed":
        raise ValueError(f"{metadata_path}: gripper_semantics must be 'closed_fraction_0_open_1_closed'")
    camera_ports = metadata["camera_ports"]
    if not isinstance(camera_ports, dict) or set(camera_ports) != {"exterior", "wrist"}:
        raise ValueError(f"{metadata_path}: camera_ports must contain exactly exterior and wrist")
    port_values = list(camera_ports.values())
    if any(type(port) is not int or not 1 <= port <= 65535 for port in port_values) or len(set(port_values)) != 2:
        raise ValueError(f"{metadata_path}: camera ports must be distinct integers in [1, 65535]")
    for digest_field in ("gello_device_sha256", "calibration_sha256"):
        if not isinstance(metadata[digest_field], str) or not SHA256_PATTERN.fullmatch(metadata[digest_field]):
            raise ValueError(f"{metadata_path}: {digest_field} must be a lowercase SHA-256 digest")
    metadata_task_value = metadata.get("task")
    if not isinstance(metadata_task_value, str):
        raise ValueError(f"{metadata_path}: task must be a string")
    metadata_task = metadata_task_value.strip()
    if not metadata_task or metadata_task != expected_task:
        raise ValueError(f"{metadata_path}: task must exactly match --task")

    frames: list[tuple[dt.datetime, dict]] = []
    for frame_path in sorted(path.glob("*.pkl")):
        with frame_path.open("rb") as stream:
            sample = pickle.load(stream)
        if not isinstance(sample, dict):
            raise ValueError(f"{frame_path}: expected a dict, got {type(sample).__name__}")
        sample_task = sample.get("task")
        if not isinstance(sample_task, str):
            raise ValueError(f"{frame_path}: task must be a string")
        if sample_task.strip() != metadata_task:
            raise ValueError(f"{frame_path}: task does not match episode.json")
        frames.append((_timestamp_from_path(frame_path), sample))
    recorded_frames = metadata.get("recorded_frames")
    if isinstance(recorded_frames, bool) or not isinstance(recorded_frames, int) or recorded_frames != len(frames):
        raise ValueError(
            f"{metadata_path}: recorded_frames={recorded_frames!r} does not match {len(frames)} pickle frames"
        )
    if any(frames[index][0] <= frames[index - 1][0] for index in range(1, len(frames))):
        raise ValueError(f"{path}: timestamps are not strictly increasing")
    if len(frames) < 2:
        raise ValueError(f"{path}: need at least two recorded frames, got {len(frames)}")

    period_s = 1.0 / TARGET_FPS
    intervals = np.asarray(
        [(frames[index][0] - frames[index - 1][0]).total_seconds() for index in range(1, len(frames))],
        dtype=np.float64,
    )
    max_gap_s = float(np.max(intervals))
    median_period_s = float(np.median(intervals))
    median_period_error_s = abs(median_period_s - period_s)
    jitter_p95_s = float(np.quantile(np.abs(intervals - period_s), 0.95))
    inferred_intervals = int(np.sum(np.maximum(1, np.rint(intervals / period_s).astype(np.int64))))
    cadence_ratio = len(intervals) / inferred_intervals
    if max_gap_s > max_frame_gap_s:
        raise ValueError(f"{path}: max frame gap {max_gap_s:.4f}s exceeds {max_frame_gap_s:.4f}s")
    if median_period_error_s > MAX_MEDIAN_PERIOD_ERROR_S:
        raise ValueError(
            f"{path}: median frame period {median_period_s:.4f}s is not {TARGET_FPS} Hz "
            f"(allowed error {MAX_MEDIAN_PERIOD_ERROR_S:.4f}s)"
        )
    if jitter_p95_s > max_period_jitter_s:
        raise ValueError(f"{path}: p95 period jitter {jitter_p95_s:.4f}s exceeds {max_period_jitter_s:.4f}s")
    if cadence_ratio < min_cadence_ratio:
        raise ValueError(f"{path}: cadence ratio {cadence_ratio:.3f} is below {min_cadence_ratio:.3f}")
    _validate_episode_commands(frames, episode_path=path)
    cadence = {
        "max_frame_gap_s": max_gap_s,
        "median_period_s": median_period_s,
        "p95_period_jitter_s": jitter_p95_s,
        "cadence_ratio": cadence_ratio,
    }
    return frames, metadata, cadence


def _validate_episode_commands(frames: list[tuple[dt.datetime, dict]], *, episode_path: Path) -> None:
    """Reconstruct the recorder's stateful joint-command safety invariants."""

    period_s = 1.0 / TARGET_FPS
    previous_velocity = np.zeros(7, dtype=np.float64)
    for frame_index, (_, sample) in enumerate(frames):
        measured = np.asarray(sample.get("joint_positions"), dtype=np.float64).reshape(-1)
        command = np.asarray(sample.get("control"), dtype=np.float64).reshape(-1)
        if measured.size < 7 or not np.isfinite(measured[:7]).all():
            raise ValueError(f"{episode_path}: frame {frame_index} has invalid joint_positions")
        if command.shape != (8,) or not np.isfinite(command).all():
            raise ValueError(f"{episode_path}: frame {frame_index} has invalid 8D control")

        command_step = command[:7] - measured[:7]
        max_step = float(np.max(np.abs(command_step)))
        if max_step > MAX_JOINT_STEP_RAD + SAFETY_STEP_ATOL_RAD:
            raise ValueError(
                f"{episode_path}: frame {frame_index} commanded joint step {max_step:.6f} rad "
                f"exceeds {MAX_JOINT_STEP_RAD:.6f} rad"
            )
        commanded_velocity = command_step / period_s
        max_acceleration = float(np.max(np.abs(commanded_velocity - previous_velocity)) / period_s)
        if max_acceleration > MAX_JOINT_ACCELERATION_RAD_S2 + SAFETY_ACCELERATION_ATOL_RAD_S2:
            raise ValueError(
                f"{episode_path}: frame {frame_index} commanded joint acceleration "
                f"{max_acceleration:.6f} rad/s^2 exceeds {MAX_JOINT_ACCELERATION_RAD_S2:.6f} rad/s^2"
            )
        previous_velocity = commanded_velocity


def _as_rgb(image: object, *, key: str, size: tuple[int, int]) -> np.ndarray:
    array = np.asarray(image)
    if array.ndim != 3 or array.shape[-1] != 3 or array.dtype != np.uint8:
        raise ValueError(f"{key} must be uint8 HWC with 3 channels, got {array.shape} {array.dtype}")
    width, height = size
    return np.asarray(Image.fromarray(array, mode="RGB").resize((width, height), Image.Resampling.LANCZOS))


def _gripper_closed_fraction(value: object, *, key: str) -> float:
    scalar = float(np.asarray(value).reshape(-1)[0])
    if not np.isfinite(scalar):
        raise ValueError(f"{key} is not finite")
    if scalar < 0.0 or scalar > 1.0:
        raise ValueError(f"{key}={scalar:.4f} is outside normalized range [0, 1]")
    return scalar


def _extract_frame(sample: dict, args: argparse.Namespace) -> dict:
    required = ("joint_positions", "gripper_position", "control", EXTERIOR_KEY, WRIST_KEY, "schema_id")
    missing = [key for key in required if key not in sample]
    if missing:
        raise ValueError(f"raw frame is missing required keys: {missing}")

    raw_joint = np.asarray(sample["joint_positions"], dtype=np.float32).reshape(-1)
    if raw_joint.size < 7 or not np.isfinite(raw_joint).all():
        raise ValueError(f"joint_positions must contain at least 7 finite values, got {raw_joint.shape}")
    q = raw_joint[:7]
    if np.any(q < JOINT_LOW) or np.any(q > JOINT_HIGH):
        raise ValueError(f"observed joint position is outside Panda limits: {q.tolist()}")

    if sample["schema_id"] != SCHEMA_ID:
        raise ValueError(f"raw frame schema_id must be {SCHEMA_ID!r}, got {sample['schema_id']!r}")
    gripper_closed = _gripper_closed_fraction(sample["gripper_position"], key="gripper_position")

    action = np.asarray(sample["control"], dtype=np.float32).reshape(-1)
    if action.shape != (8,) or not np.isfinite(action).all():
        raise ValueError(f"control must contain exactly 8 finite values, got {action.shape}")
    if np.any(action[:7] < JOINT_LOW + JOINT_MARGIN_RAD) or np.any(action[:7] > JOINT_HIGH - JOINT_MARGIN_RAD):
        raise ValueError(f"commanded joint target is outside conservative Panda limits: {action[:7].tolist()}")
    action = action.copy()
    action[7] = _gripper_closed_fraction(action[7], key="control[7]")

    task_value = sample.get("task", args.task)
    if not isinstance(task_value, str):
        raise ValueError("task prompt must be a string")
    task = task_value.strip()
    if not task:
        raise ValueError("task prompt must not be empty")

    return {
        "observation.images.exterior": _as_rgb(
            sample[EXTERIOR_KEY],
            key=EXTERIOR_KEY,
            size=(args.width, args.height),
        ),
        "observation.images.wrist": _as_rgb(
            sample[WRIST_KEY],
            key=WRIST_KEY,
            size=(args.width, args.height),
        ),
        "observation.state": np.concatenate([q, np.array([gripper_closed], dtype=np.float32)]),
        "action": action,
        "task": task,
    }


def _features(args: argparse.Namespace) -> dict:
    return {
        "observation.images.exterior": {
            "dtype": "image",
            "shape": (args.height, args.width, 3),
            "names": ["height", "width", "channel"],
        },
        "observation.images.wrist": {
            "dtype": "image",
            "shape": (args.height, args.width, 3),
            "names": ["height", "width", "channel"],
        },
        "observation.state": {
            "dtype": "float32",
            "shape": (8,),
            "names": ["q1", "q2", "q3", "q4", "q5", "q6", "q7", "gripper_closed"],
        },
        "action": {
            "dtype": "float32",
            "shape": (8,),
            "names": ["q1", "q2", "q3", "q4", "q5", "q6", "q7", "gripper_closed"],
        },
    }


def _write_final_manifest(output_path: Path, manifest: dict) -> Path:
    """Bind the finalized payload and atomically publish its root manifest."""

    expected_input_keys = FRANKA_DATASET_MANIFEST_KEYS - {"content_digest"}
    actual_input_keys = set(manifest)
    if actual_input_keys != expected_input_keys:
        missing = sorted(expected_input_keys - actual_input_keys)
        unknown = sorted(actual_input_keys - expected_input_keys)
        raise ValueError(
            f"dataset manifest fields must exactly match the contract; missing={missing}, unknown={unknown}"
        )
    finalized = dict(manifest)
    finalized["content_digest"] = compute_dataset_content_digest(output_path).to_mapping()
    destination = output_path / "franka_manifest.json"
    temporary = output_path / "franka_manifest.json.tmp"
    temporary.write_text(json.dumps(finalized, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    temporary.replace(destination)
    return destination


def parse_args() -> argparse.Namespace:
    from lerobot.common.datasets.lerobot_dataset import HF_LEROBOT_HOME

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-dir", type=Path, required=True)
    parser.add_argument("--task", required=True)
    parser.add_argument("--repo-id", default=DEFAULT_REPO_ID)
    parser.add_argument("--dataset-home", type=Path, default=Path(HF_LEROBOT_HOME))
    parser.add_argument("--target-fps", type=int, default=TARGET_FPS)
    parser.add_argument("--width", type=int, default=320)
    parser.add_argument("--height", type=int, default=240)
    parser.add_argument("--max-frame-gap-s", type=float, default=MAX_FRAME_GAP_S)
    parser.add_argument("--max-period-jitter-s", type=float, default=MAX_PERIOD_JITTER_S)
    parser.add_argument("--min-cadence-ratio", type=float, default=MIN_CADENCE_RATIO)
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--push-to-hub", action="store_true")
    parser.add_argument("--public", action="store_true", help="Make a pushed dataset public; private is the default")
    parser.add_argument(
        "--dataset-license",
        help="Optional dataset license identifier; never inferred from this repository's code license",
    )
    return parser.parse_args()


def main() -> None:
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

    args = parse_args()
    if args.target_fps != TARGET_FPS:
        raise ValueError(f"target-fps is fixed by the policy contract at {TARGET_FPS}")
    if args.width <= 0 or args.height <= 0:
        raise ValueError("width and height must be positive")
    if not 1.0 / TARGET_FPS < args.max_frame_gap_s <= MAX_FRAME_GAP_S:
        raise ValueError(f"max-frame-gap-s must be in ({1.0 / TARGET_FPS}, {MAX_FRAME_GAP_S}]")
    if not 0.0 < args.max_period_jitter_s <= MAX_PERIOD_JITTER_S:
        raise ValueError(f"max-period-jitter-s must be in (0, {MAX_PERIOD_JITTER_S}]")
    if not MIN_CADENCE_RATIO <= args.min_cadence_ratio <= 1.0:
        raise ValueError(f"min-cadence-ratio must be in [{MIN_CADENCE_RATIO}, 1]")
    if args.public and not args.push_to_hub:
        raise ValueError("--public is only meaningful together with --push-to-hub")
    if args.dataset_license is not None:
        args.dataset_license = args.dataset_license.strip()
        if not args.dataset_license:
            raise ValueError("--dataset-license must not be empty")
        if not args.push_to_hub:
            raise ValueError("--dataset-license is only meaningful together with --push-to-hub")
    raw_dir = args.raw_dir.expanduser().resolve()
    if not raw_dir.is_dir():
        raise FileNotFoundError(raw_dir)

    dataset_home = args.dataset_home.expanduser().resolve()
    output_path = _resolve_output_path(dataset_home=dataset_home, repo_id=args.repo_id, raw_dir=raw_dir)
    if output_path.exists():
        if not args.overwrite:
            raise FileExistsError(f"dataset already exists: {output_path}; pass --overwrite to replace it")
        _validate_owned_overwrite_target(output_path, repo_id=args.repo_id)
        shutil.rmtree(output_path)

    episode_dirs = [path for path in sorted(raw_dir.iterdir()) if path.is_dir() and not path.name.startswith(".")]
    if not episode_dirs:
        raise ValueError(f"no episode directories found in {raw_dir}")

    dataset = LeRobotDataset.create(
        repo_id=args.repo_id,
        root=output_path,
        robot_type="franka_panda",
        fps=args.target_fps,
        features=_features(args),
        image_writer_threads=8,
        image_writer_processes=0,
    )

    summaries = []
    try:
        for episode_dir in episode_dirs:
            raw_frames, episode_metadata, cadence = _load_episode(
                episode_dir,
                expected_task=args.task.strip(),
                max_frame_gap_s=args.max_frame_gap_s,
                max_period_jitter_s=args.max_period_jitter_s,
                min_cadence_ratio=args.min_cadence_ratio,
            )
            # The recorder and metadata contract are already fixed at 20 Hz,
            # and _load_episode rejects gaps, jitter, or inferred drops. Keep
            # every audited frame: threshold resampling can turn harmless
            # +/-1 ms jitter into an accidental 10 Hz sequence.
            frames = raw_frames
            for _, sample in frames:
                dataset.add_frame(_extract_frame(sample, args))
            dataset.save_episode()
            summaries.append(
                {
                    "episode": episode_dir.name,
                    "outcome": episode_metadata["outcome"],
                    "raw_frames": len(raw_frames),
                    "frames": len(frames),
                    "duration_s": (raw_frames[-1][0] - raw_frames[0][0]).total_seconds(),
                    **cadence,
                }
            )
            print(f"[converted] {episode_dir.name}: {len(raw_frames)} -> {len(frames)} frames")
    finally:
        dataset.stop_image_writer()

    manifest = {
        "schema_id": SCHEMA_ID,
        "repo_id": args.repo_id,
        "task": args.task.strip(),
        "fps": args.target_fps,
        "color_space": "rgb_uint8_hwc",
        "state": "q7_rad + gripper_closed",
        "action": "absolute_q7_rad + gripper_closed",
        "camera_roles": {"exterior": EXTERIOR_KEY, "wrist": WRIST_KEY},
        "episodes": summaries,
    }
    _write_final_manifest(output_path, manifest)
    print(f"[done] dataset={output_path} episodes={len(summaries)}")

    if args.push_to_hub:
        dataset.push_to_hub(
            tags=["franka", "panda", "gello", SCHEMA_ID],
            private=not args.public,
            push_videos=True,
            license=args.dataset_license,
        )


if __name__ == "__main__":
    main()
