#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
PROGRAM_NAME="${0##*/}"
GELLO_REPOSITORY="https://github.com/wuphilipp/gello_software.git"
GELLO_COMMIT="204f53a64bef89471a1e483b0f874f755fbd2d3a"

usage() {
    cat <<EOF
Usage: ${PROGRAM_NAME} --destination PATH [--python VERSION] [--source-only]

Clone GELLO at the repository's audited pin and install it into a local uv
virtual environment. Re-running at the same pin is safe. A dirty checkout is
never reset or overwritten.

Options:
  --destination PATH  Absolute persistent checkout path.
  --python VERSION    Python 3.11 interpreter requested from uv (default: 3.11).
  --source-only       Clone/verify the pinned source without creating a venv.
  -h, --help          Show this help.
EOF
}

destination="${GELLO_INSTALL_ROOT:-}"
python_version="${GELLO_PYTHON_VERSION:-3.11}"
source_only=false

while (($#)); do
    case "$1" in
        --destination) destination="${2:?missing value for --destination}"; shift 2 ;;
        --python) python_version="${2:?missing value for --python}"; shift 2 ;;
        --source-only) source_only=true; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "ERROR: unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

[[ "$destination" == /* ]] || { echo "ERROR: --destination must be an absolute path" >&2; exit 2; }
[[ "$python_version" == "3.11" || "$python_version" == 3.11.* ]] || {
    echo "ERROR: this pinned GELLO runtime requires Python 3.11; requested ${python_version}" >&2
    exit 2
}
case "$destination" in
    /tmp|/tmp/*|/var/tmp|/var/tmp/*) echo "ERROR: destination must be persistent" >&2; exit 2 ;;
esac
for command_name in git; do
    command -v "$command_name" >/dev/null 2>&1 || { echo "ERROR: required command not found: $command_name" >&2; exit 3; }
done

if [[ -e "$destination" && ! -d "$destination/.git" ]]; then
    echo "ERROR: destination exists but is not a Git checkout: $destination" >&2
    exit 1
fi

if [[ ! -d "$destination/.git" ]]; then
    mkdir -p "$(dirname "$destination")"
    git clone --filter=blob:none "$GELLO_REPOSITORY" "$destination"
fi

origin_url="$(git -C "$destination" remote get-url origin)"
[[ "$origin_url" == "$GELLO_REPOSITORY" || "$origin_url" == "git@github.com:wuphilipp/gello_software.git" ]] || {
    echo "ERROR: unexpected GELLO origin: $origin_url" >&2
    exit 1
}

current_commit="$(git -C "$destination" rev-parse HEAD 2>/dev/null || true)"
if [[ "$current_commit" != "$GELLO_COMMIT" ]]; then
    [[ -z "$(git -C "$destination" status --porcelain)" ]] || {
        echo "ERROR: GELLO checkout is dirty; refusing to switch commits" >&2
        exit 1
    }
    git -C "$destination" fetch --depth 1 origin "$GELLO_COMMIT"
    git -C "$destination" switch --detach "$GELLO_COMMIT"
fi

actual_commit="$(git -C "$destination" rev-parse HEAD)"
[[ "$actual_commit" == "$GELLO_COMMIT" ]] || { echo "ERROR: failed to select GELLO pin" >&2; exit 1; }
[[ -z "$(git -C "$destination" status --porcelain --untracked-files=no)" ]] || {
    echo "ERROR: GELLO checkout has tracked modifications; refusing to install an unverified tree" >&2
    exit 1
}
git -C "$destination" submodule update --init --recursive third_party/DynamixelSDK
echo "GELLO source verified at ${actual_commit}"

if [[ "$source_only" == true ]]; then
    exit 0
fi

command -v uv >/dev/null 2>&1 || { echo "ERROR: uv not found; install uv before GELLO" >&2; exit 3; }
[[ "$(uname -s)" == "Linux" && "$(uname -m)" == "x86_64" ]] || {
    echo "ERROR: the pinned GELLO/RealSense runtime is gated only for Linux x86_64" >&2
    exit 1
}
venv="$destination/.venv"
export UV_CACHE_DIR="$destination/.uv-cache"
export TMPDIR="$destination/.tmp"
export TEMP="$TMPDIR"
export TMP="$TMPDIR"
mkdir -p "$UV_CACHE_DIR" "$TMPDIR"
if [[ ! -x "$venv/bin/python" ]]; then
    uv venv --python "$python_version" "$venv"
fi
actual_python_version="$("$venv/bin/python" -c 'import platform; print(platform.python_version())')"
[[ "$actual_python_version" == 3.11.* ]] || {
    echo "ERROR: GELLO virtual environment must use Python 3.11, found ${actual_python_version}" >&2
    echo "Remove or move the incompatible ${venv} explicitly, then rerun this installer." >&2
    exit 1
}
runtime_requirements="$SCRIPT_DIR/../../deploy/gello/requirements-franka-py311.txt"
[[ -f "$runtime_requirements" ]] || { echo "ERROR: pinned runtime requirements missing: $runtime_requirements" >&2; exit 1; }
franka_runtime_package="$PROJECT_ROOT/packages/franka-runtime"
[[ -f "$franka_runtime_package/pyproject.toml" ]] || {
    echo "ERROR: franka-runtime package is missing from the selected checkout: $franka_runtime_package" >&2
    exit 1
}
uv pip install --python "$venv/bin/python" --requirement "$runtime_requirements"
uv pip install --python "$venv/bin/python" --no-deps --editable "$destination"
uv pip install --python "$venv/bin/python" --no-deps --editable "$destination/third_party/DynamixelSDK/python"
uv pip install --python "$venv/bin/python" --no-deps --editable "$franka_runtime_package"
uv pip check --python "$venv/bin/python"
PROJECT_ROOT="$PROJECT_ROOT" "$venv/bin/python" - <<'PY'
import os
from pathlib import Path

import cv2
import evdev
import franka_runtime
import gello
import pyrealsense2
import serial
import tyro
import zmq

project_root = Path(os.environ["PROJECT_ROOT"]).resolve()
runtime_path = Path(franka_runtime.__file__).resolve()
if project_root not in runtime_path.parents:
    raise SystemExit(f"franka_runtime is not loaded from the selected checkout: {runtime_path}")
print("GELLO Franka runtime imports passed")
PY

echo "GELLO installed at ${destination}; interpreter: ${venv}/bin/python (${actual_python_version})"
