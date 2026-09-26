"""Deterministic integrity contract for a Franka LeRobot v2.1 dataset."""

from __future__ import annotations

import hashlib
import json
import os
import re
import stat
from dataclasses import asdict, dataclass
from pathlib import Path, PurePosixPath
from typing import Any, Mapping

from franka_runtime.contracts import ContractError

DATASET_CONTENT_SCHEMA_VERSION = "franka-dataset-content/v1"
DATASET_CONTENT_ALGORITHM = "sha256"
LEROBOT_CODEBASE_VERSION = "v2.1"
LEROBOT_DATA_PATH = "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet"
LEROBOT_VIDEO_PATH = "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4"
FRANKA_DATASET_MANIFEST_KEYS = frozenset(
    {
        "schema_id",
        "repo_id",
        "task",
        "fps",
        "color_space",
        "state",
        "action",
        "camera_roles",
        "episodes",
        "content_digest",
    }
)

_REQUIRED_META_FILES = frozenset(
    {
        "meta/info.json",
        "meta/episodes.jsonl",
        "meta/episodes_stats.jsonl",
        "meta/tasks.jsonl",
    }
)
_SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")
_STREAM_CHUNK_BYTES = 1024 * 1024


class DatasetContentError(ContractError):
    """Raised when dataset bytes do not satisfy the closed content contract."""


def _require_plain_nonnegative_int(name: str, value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise DatasetContentError(f"{name} must be a non-negative integer")
    return value


@dataclass(frozen=True)
class DatasetContentDigest:
    """Closed manifest schema for the complete training payload."""

    schema_version: str
    algorithm: str
    sha256: str
    file_count: int
    total_bytes: int

    def __post_init__(self) -> None:
        if self.schema_version != DATASET_CONTENT_SCHEMA_VERSION:
            raise DatasetContentError(
                f"content_digest.schema_version must be {DATASET_CONTENT_SCHEMA_VERSION!r}, got {self.schema_version!r}"
            )
        if self.algorithm != DATASET_CONTENT_ALGORITHM:
            raise DatasetContentError(
                f"content_digest.algorithm must be {DATASET_CONTENT_ALGORITHM!r}, got {self.algorithm!r}"
            )
        if not isinstance(self.sha256, str) or _SHA256_PATTERN.fullmatch(self.sha256) is None:
            raise DatasetContentError("content_digest.sha256 must be a lowercase SHA-256 hex digest")
        file_count = _require_plain_nonnegative_int("content_digest.file_count", self.file_count)
        _require_plain_nonnegative_int("content_digest.total_bytes", self.total_bytes)
        if file_count == 0:
            raise DatasetContentError("content_digest.file_count must be positive")

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> DatasetContentDigest:
        if not isinstance(value, Mapping):
            raise DatasetContentError(f"content_digest must be a mapping, got {type(value).__name__}")
        expected_keys = {field_name for field_name in cls.__dataclass_fields__}
        actual_keys = set(value)
        missing = sorted(expected_keys - actual_keys)
        unknown = sorted(repr(key) for key in actual_keys - expected_keys)
        if missing or unknown:
            parts = []
            if missing:
                parts.append(f"missing keys: {missing}")
            if unknown:
                parts.append(f"unknown keys: {unknown}")
            raise DatasetContentError("invalid content_digest (" + "; ".join(parts) + ")")
        return cls(
            schema_version=value["schema_version"],
            algorithm=value["algorithm"],
            sha256=value["sha256"],
            file_count=value["file_count"],
            total_bytes=value["total_bytes"],
        )

    def to_mapping(self) -> dict[str, object]:
        return asdict(self)


def _path_exists(path: Path) -> bool:
    try:
        path.lstat()
    except FileNotFoundError:
        return False
    except OSError as exc:
        raise DatasetContentError(f"cannot inspect dataset path {path}: {exc}") from exc
    return True


def _require_directory(path: Path, *, label: str) -> None:
    try:
        status = path.lstat()
    except OSError as exc:
        raise DatasetContentError(f"cannot inspect required {label} directory {path}: {exc}") from exc
    if stat.S_ISLNK(status.st_mode):
        raise DatasetContentError(f"{label} directory must not be a symbolic link: {path}")
    if not stat.S_ISDIR(status.st_mode):
        raise DatasetContentError(f"required {label} path is not a directory: {path}")


def _read_regular_file(path: Path, *, label: str) -> bytes:
    try:
        status = path.lstat()
    except OSError as exc:
        raise DatasetContentError(f"cannot inspect required {label} file {path}: {exc}") from exc
    if stat.S_ISLNK(status.st_mode):
        raise DatasetContentError(f"{label} file must not be a symbolic link: {path}")
    if not stat.S_ISREG(status.st_mode):
        raise DatasetContentError(f"required {label} path is not a regular file: {path}")
    try:
        return path.read_bytes()
    except OSError as exc:
        raise DatasetContentError(f"cannot read required {label} file {path}: {exc}") from exc


def _load_info(root: Path) -> tuple[dict[str, Any], bytes]:
    path = root / "meta" / "info.json"
    raw = _read_regular_file(path, label="LeRobot info")
    try:
        value = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise DatasetContentError(f"LeRobot info is not valid UTF-8 JSON: {path}") from exc
    if not isinstance(value, dict):
        raise DatasetContentError("LeRobot meta/info.json must contain a JSON object")
    return value, raw


def _safe_video_key(value: object) -> str:
    if not isinstance(value, str) or not value or "\\" in value:
        raise DatasetContentError(f"invalid video feature key: {value!r}")
    path = PurePosixPath(value)
    if path.is_absolute() or len(path.parts) != 1 or path.parts[0] in {".", ".."}:
        raise DatasetContentError(f"invalid video feature key: {value!r}")
    return value


def _expected_payload_paths(info: Mapping[str, Any]) -> set[str]:
    if info.get("codebase_version") != LEROBOT_CODEBASE_VERSION:
        raise DatasetContentError(
            f"meta/info.json codebase_version must be {LEROBOT_CODEBASE_VERSION!r}, "
            f"got {info.get('codebase_version')!r}"
        )
    chunks_size = info.get("chunks_size")
    if isinstance(chunks_size, bool) or not isinstance(chunks_size, int) or chunks_size < 1:
        raise DatasetContentError("meta/info.json chunks_size must be a positive integer")
    total_episodes = info.get("total_episodes")
    if isinstance(total_episodes, bool) or not isinstance(total_episodes, int) or total_episodes < 1:
        raise DatasetContentError("meta/info.json total_episodes must be a positive integer")
    if info.get("data_path") != LEROBOT_DATA_PATH:
        raise DatasetContentError(
            f"meta/info.json data_path must be the pinned LeRobot v2.1 layout {LEROBOT_DATA_PATH!r}"
        )

    features = info.get("features")
    if not isinstance(features, dict) or not features:
        raise DatasetContentError("meta/info.json features must be a non-empty object")
    video_keys = []
    for key, feature in features.items():
        if not isinstance(feature, dict):
            raise DatasetContentError(f"meta/info.json feature {key!r} must be an object")
        if feature.get("dtype") == "video":
            video_keys.append(_safe_video_key(key))

    video_path = info.get("video_path")
    if video_path not in {None, LEROBOT_VIDEO_PATH}:
        raise DatasetContentError(
            f"meta/info.json video_path must be null or the pinned LeRobot v2.1 layout {LEROBOT_VIDEO_PATH!r}"
        )
    if video_keys and video_path != LEROBOT_VIDEO_PATH:
        raise DatasetContentError("video features require the pinned LeRobot v2.1 video_path layout")

    expected = set(_REQUIRED_META_FILES)
    for episode_index in range(total_episodes):
        episode_chunk = episode_index // chunks_size
        expected.add(LEROBOT_DATA_PATH.format(episode_chunk=episode_chunk, episode_index=episode_index))
        for video_key in video_keys:
            expected.add(
                LEROBOT_VIDEO_PATH.format(
                    episode_chunk=episode_chunk,
                    episode_index=episode_index,
                    video_key=video_key,
                )
            )
    return expected


def _expected_directories(files: set[str]) -> set[str]:
    directories: set[str] = set()
    for relative in files:
        parent = PurePosixPath(relative).parent
        while parent != PurePosixPath("."):
            directories.add(parent.as_posix())
            parent = parent.parent
    return directories


def _scan_payload(root: Path, *, include_videos: bool) -> tuple[set[str], set[str]]:
    files: set[str] = set()
    directories: set[str] = set()
    roots = ["meta", "data"]
    if include_videos:
        roots.append("videos")
    elif _path_exists(root / "videos"):
        raise DatasetContentError("videos/ is forbidden when meta/info.json declares no video features")
    if _path_exists(root / "images"):
        raise DatasetContentError("temporary images/ payload must not remain in a finalized LeRobot dataset")

    for name in roots:
        payload_root = root / name
        _require_directory(payload_root, label=name)
        directories.add(name)
        for current, dirnames, filenames in os.walk(str(payload_root), topdown=True, followlinks=False):
            dirnames.sort()
            filenames.sort()
            current_path = Path(current)
            for dirname in dirnames:
                path = current_path / dirname
                try:
                    status = path.lstat()
                except OSError as exc:
                    raise DatasetContentError(f"cannot inspect dataset directory {path}: {exc}") from exc
                if stat.S_ISLNK(status.st_mode):
                    raise DatasetContentError(f"dataset payload must not contain symbolic links: {path}")
                if not stat.S_ISDIR(status.st_mode):
                    raise DatasetContentError(f"unsupported dataset payload directory entry: {path}")
                directories.add(path.relative_to(root).as_posix())
            for filename in filenames:
                path = current_path / filename
                try:
                    status = path.lstat()
                except OSError as exc:
                    raise DatasetContentError(f"cannot inspect dataset file {path}: {exc}") from exc
                if stat.S_ISLNK(status.st_mode):
                    raise DatasetContentError(f"dataset payload must not contain symbolic links: {path}")
                if not stat.S_ISREG(status.st_mode):
                    raise DatasetContentError(f"dataset payload must contain only regular files: {path}")
                files.add(path.relative_to(root).as_posix())
    return files, directories


def _stat_identity(status: os.stat_result) -> tuple[int, int, int, int, int]:
    return (status.st_mode, status.st_dev, status.st_ino, status.st_size, status.st_mtime_ns)


def _hash_file(digest: Any, root: Path, relative: str) -> int:
    path = root.joinpath(*PurePosixPath(relative).parts)
    try:
        initial = path.lstat()
    except OSError as exc:
        raise DatasetContentError(f"cannot inspect dataset payload file {path}: {exc}") from exc
    if stat.S_ISLNK(initial.st_mode) or not stat.S_ISREG(initial.st_mode):
        raise DatasetContentError(f"dataset payload path is not a regular non-symlink file: {path}")

    relative_bytes = relative.encode("utf-8")
    digest.update(len(relative_bytes).to_bytes(8, byteorder="big"))
    digest.update(relative_bytes)
    digest.update(initial.st_size.to_bytes(8, byteorder="big"))

    flags = os.O_RDONLY | getattr(os, "O_BINARY", 0) | getattr(os, "O_NOFOLLOW", 0)
    bytes_read = 0
    try:
        descriptor = os.open(str(path), flags)
        with os.fdopen(descriptor, "rb") as stream:
            opened = os.fstat(stream.fileno())
            if _stat_identity(opened) != _stat_identity(initial):
                raise DatasetContentError(f"dataset payload changed while hashing: {relative}")
            for chunk in iter(lambda: stream.read(_STREAM_CHUNK_BYTES), b""):
                bytes_read += len(chunk)
                digest.update(chunk)
            finished = os.fstat(stream.fileno())
    except DatasetContentError:
        raise
    except OSError as exc:
        raise DatasetContentError(f"cannot hash dataset payload file {path}: {exc}") from exc
    try:
        final = path.lstat()
    except OSError as exc:
        raise DatasetContentError(f"dataset payload changed while hashing: {relative}: {exc}") from exc
    if (
        bytes_read != initial.st_size
        or _stat_identity(opened) != _stat_identity(finished)
        or _stat_identity(initial) != _stat_identity(final)
    ):
        raise DatasetContentError(f"dataset payload changed while hashing: {relative}")
    return bytes_read


def compute_dataset_content_digest(dataset_root: os.PathLike[str] | str) -> DatasetContentDigest:
    """Stream-hash the exact finalized LeRobot v2.1 files used for training."""

    root = Path(dataset_root).expanduser()
    _require_directory(root, label="dataset root")
    info, initial_info_bytes = _load_info(root)
    expected_files = _expected_payload_paths(info)
    expected_directories = _expected_directories(expected_files)
    include_videos = any(relative.startswith("videos/") for relative in expected_files)
    actual_files, actual_directories = _scan_payload(root, include_videos=include_videos)
    missing_files = sorted(expected_files - actual_files)
    unknown_files = sorted(actual_files - expected_files)
    missing_directories = sorted(expected_directories - actual_directories)
    unknown_directories = sorted(actual_directories - expected_directories)
    if missing_files or unknown_files or missing_directories or unknown_directories:
        raise DatasetContentError(
            "dataset payload layout does not match the closed LeRobot v2.1 contract; "
            f"missing_files={missing_files}, unknown_files={unknown_files}, "
            f"missing_directories={missing_directories}, unknown_directories={unknown_directories}"
        )

    digest = hashlib.sha256()
    digest.update(b"franka-dataset-content-v1\0")
    total_bytes = 0
    sorted_files = sorted(expected_files, key=lambda value: value.encode("utf-8"))
    for relative in sorted_files:
        total_bytes += _hash_file(digest, root, relative)

    final_files, final_directories = _scan_payload(root, include_videos=include_videos)
    if final_files != actual_files or final_directories != actual_directories:
        raise DatasetContentError("dataset payload layout changed while hashing")
    _final_info, final_info_bytes = _load_info(root)
    if final_info_bytes != initial_info_bytes:
        raise DatasetContentError("meta/info.json changed while hashing the dataset payload")
    return DatasetContentDigest(
        schema_version=DATASET_CONTENT_SCHEMA_VERSION,
        algorithm=DATASET_CONTENT_ALGORITHM,
        sha256=digest.hexdigest(),
        file_count=len(sorted_files),
        total_bytes=total_bytes,
    )


def verify_dataset_content_digest(
    dataset_root: os.PathLike[str] | str,
    expected: Mapping[str, Any] | DatasetContentDigest,
) -> DatasetContentDigest:
    """Recompute a dataset digest and fail unless every bound value matches."""

    expected_digest = (
        expected if isinstance(expected, DatasetContentDigest) else DatasetContentDigest.from_mapping(expected)
    )
    actual_digest = compute_dataset_content_digest(dataset_root)
    if actual_digest != expected_digest:
        raise DatasetContentError(
            "dataset content digest mismatch: "
            f"expected sha256={expected_digest.sha256} files={expected_digest.file_count} "
            f"bytes={expected_digest.total_bytes}, got sha256={actual_digest.sha256} "
            f"files={actual_digest.file_count} bytes={actual_digest.total_bytes}"
        )
    return actual_digest
