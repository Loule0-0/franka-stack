#!/usr/bin/env bash
set -Eeuo pipefail

PROGRAM_NAME="${0##*/}"
UV_VERSION_DEFAULT="0.11.12"

usage() {
    cat <<EOF
Usage: ${PROGRAM_NAME} --persistent-root PATH --project-root PATH \\
  --dataset-root PATH --checkpoint-root PATH [options]

Install uv under persistent storage, install CPython 3.11, and synchronize the
pinned OpenPI submodule. The script redirects temporary files and caches away
from /tmp, then installs the parent franka-runtime package into that environment.

Options:
  --persistent-root PATH  Account-owned persistent root.
  --project-root PATH     Existing franka-stack checkout.
  --dataset-root PATH     Persistent dataset directory.
  --checkpoint-root PATH  Persistent checkpoint directory.
  --wheelhouse PATH       Add local resolver/build artifacts; locked URLs still use the uv cache.
  --offline               Forbid network access; requires a fully prewarmed uv cache.
  --uv-version VERSION    uv installer version (default: ${UV_VERSION_DEFAULT}).
  -h, --help              Show this help.

The same values may be provided through FRANKA_PERSISTENT_ROOT,
FRANKA_PROJECT_ROOT, HF_LEROBOT_HOME, FRANKA_CHECKPOINT_ROOT, and
FRANKA_WHEELHOUSE.
EOF
}

persistent_root="${FRANKA_PERSISTENT_ROOT:-}"
project_root="${FRANKA_PROJECT_ROOT:-}"
dataset_root="${HF_LEROBOT_HOME:-}"
checkpoint_root="${FRANKA_CHECKPOINT_ROOT:-}"
wheelhouse="${FRANKA_WHEELHOUSE:-}"
uv_version="${UV_VERSION:-$UV_VERSION_DEFAULT}"
offline=false

while (($#)); do
    case "$1" in
        --persistent-root) persistent_root="${2:?missing value for --persistent-root}"; shift 2 ;;
        --project-root) project_root="${2:?missing value for --project-root}"; shift 2 ;;
        --dataset-root) dataset_root="${2:?missing value for --dataset-root}"; shift 2 ;;
        --checkpoint-root) checkpoint_root="${2:?missing value for --checkpoint-root}"; shift 2 ;;
        --wheelhouse) wheelhouse="${2:?missing value for --wheelhouse}"; shift 2 ;;
        --offline) offline=true; shift ;;
        --uv-version) uv_version="${2:?missing value for --uv-version}"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "ERROR: unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

require_persistent_absolute_path() {
    local label="$1"
    local path="$2"
    [[ -n "$path" ]] || { echo "ERROR: ${label} is required" >&2; exit 2; }
    [[ "$path" == /* ]] || { echo "ERROR: ${label} must be absolute: $path" >&2; exit 2; }
    case "$path" in
        /tmp|/tmp/*|/var/tmp|/var/tmp/*)
            echo "ERROR: ${label} must be persistent, not under a temporary directory: $path" >&2
            exit 2
            ;;
    esac
}

require_persistent_absolute_path "persistent root" "$persistent_root"
require_persistent_absolute_path "project root" "$project_root"
require_persistent_absolute_path "dataset root" "$dataset_root"
require_persistent_absolute_path "checkpoint root" "$checkpoint_root"
openpi_root="$project_root/third_party/openpi"
[[ -f "$project_root/pyproject.toml" && -f "$openpi_root/pyproject.toml" && -f "$openpi_root/uv.lock" ]] || {
    echo "ERROR: project root or pinned OpenPI submodule is incomplete; run: git submodule update --init third_party/openpi" >&2
    exit 1
}
if [[ -n "$wheelhouse" ]]; then
    require_persistent_absolute_path "wheelhouse" "$wheelhouse"
    [[ -d "$wheelhouse" ]] || { echo "ERROR: wheelhouse not found: $wheelhouse" >&2; exit 1; }
fi

command -v awk >/dev/null 2>&1 || { echo "ERROR: required command not found: awk" >&2; exit 3; }

runtime_root="$persistent_root/.franka-openpi"
uv_bin_dir="$runtime_root/bin"
export UV_CACHE_DIR="$runtime_root/uv-cache"
export UV_PYTHON_INSTALL_DIR="$runtime_root/python"
export XDG_CACHE_HOME="$runtime_root/cache"
export TMPDIR="$runtime_root/tmp"
export TEMP="$TMPDIR"
export TMP="$TMPDIR"
mkdir -p "$uv_bin_dir" "$UV_CACHE_DIR" "$UV_PYTHON_INSTALL_DIR" "$XDG_CACHE_HOME" "$TMPDIR" "$dataset_root" "$checkpoint_root"

uv_bin="$uv_bin_dir/uv"
if [[ ! -x "$uv_bin" ]]; then
    if [[ "$offline" == true ]]; then
        echo "ERROR: --offline requires a preinstalled ${uv_bin}" >&2
        exit 1
    fi
    for command_name in curl sh; do
        command -v "$command_name" >/dev/null 2>&1 || { echo "ERROR: required command not found: $command_name" >&2; exit 3; }
    done
    echo "Installing uv ${uv_version} into ${uv_bin_dir}"
    curl -LsSf "https://astral.sh/uv/${uv_version}/install.sh" | env UV_INSTALL_DIR="$uv_bin_dir" INSTALLER_NO_MODIFY_PATH=1 sh
fi
[[ -x "$uv_bin" ]] || { echo "ERROR: uv installer did not create ${uv_bin}" >&2; exit 1; }

actual_uv_version="$("$uv_bin" --version | awk '{print $2}')"
[[ "$actual_uv_version" == "$uv_version" ]] || {
    echo "ERROR: existing uv is ${actual_uv_version}, expected ${uv_version}; replace it explicitly" >&2
    exit 1
}
"$uv_bin" --version
if [[ "$offline" == true ]]; then
    "$uv_bin" python find --managed-python --no-python-downloads --offline 3.11 >/dev/null 2>&1 || {
        echo "ERROR: --offline requires a preinstalled uv-managed Python 3.11" >&2
        exit 1
    }
else
    "$uv_bin" python install 3.11
fi

sync_args=(sync --project "$openpi_root" --frozen --python 3.11 --managed-python --no-python-downloads)
if [[ -n "$wheelhouse" ]]; then
    sync_args+=(--find-links "$wheelhouse")
fi
if [[ "$offline" == true ]]; then
    sync_args+=(--offline)
fi
"$uv_bin" "${sync_args[@]}"
"$uv_bin" pip install --python "$openpi_root/.venv/bin/python" --no-deps --editable "$project_root/packages/franka-runtime"

echo "GPU environment is synchronized from the pinned OpenPI lockfile."
echo "uv: ${uv_bin}"
echo "project: ${project_root}"
echo "openpi: ${openpi_root}"
echo "datasets: ${dataset_root}"
echo "checkpoints: ${checkpoint_root}"
echo "For later shells: export PATH=\"${uv_bin_dir}:\$PATH\""
