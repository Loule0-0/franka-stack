#!/usr/bin/env bash
set -Eeuo pipefail

PROGRAM_NAME="${0##*/}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REQUIREMENTS="$SCRIPT_DIR/../../deploy/panda-polymetis/requirements-robot-client-py38.txt"

usage() {
    cat <<EOF
Usage: ${PROGRAM_NAME} --project-root PATH --env-prefix PATH [--check]

Install the thin Franka policy/bridge clients into the exact Python 3.8
environment built by install_legacy_polymetis.sh. This does not install or
start a robot service.

Options:
  --project-root PATH  Absolute franka-stack checkout path.
  --env-prefix PATH    Absolute pinned Polymetis environment prefix.
  --check              Verify only; do not install packages.
  -h, --help           Show this help.

Environment equivalents: FRANKA_PROJECT_ROOT and PANDA_POLYMETIS_ENV_PREFIX.
EOF
}

project_root="${FRANKA_PROJECT_ROOT:-}"
env_prefix="${PANDA_POLYMETIS_ENV_PREFIX:-}"
check_only=false

while (($#)); do
    case "$1" in
        --project-root) project_root="${2:?missing value for --project-root}"; shift 2 ;;
        --env-prefix) env_prefix="${2:?missing value for --env-prefix}"; shift 2 ;;
        --check) check_only=true; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "ERROR: unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

require_persistent_absolute_path() {
    local label="$1"
    local path="$2"
    [[ -n "$path" && "$path" == /* ]] || { echo "ERROR: ${label} must be an absolute path" >&2; exit 2; }
    case "$path" in
        /tmp|/tmp/*|/var/tmp|/var/tmp/*) echo "ERROR: ${label} must be persistent: $path" >&2; exit 2 ;;
    esac
}

require_persistent_absolute_path "project root" "$project_root"
require_persistent_absolute_path "environment prefix" "$env_prefix"
for command_name in realpath sha256sum uname; do
    command -v "$command_name" >/dev/null 2>&1 || { echo "ERROR: required command not found: $command_name" >&2; exit 3; }
done
project_root="$(realpath -m -- "$project_root")"
env_prefix="$(realpath -m -- "$env_prefix")"
case "$env_prefix" in
    /|/bin|/bin/*|/boot|/boot/*|/dev|/dev/*|/etc|/etc/*|/home|/home/data|/lib|/lib/*|/lib64|/lib64/*|/opt|/proc|/proc/*|/root|/root/*|/run|/run/*|/sbin|/sbin/*|/sys|/sys/*|/usr|/usr/*|/var|/var/*)
        echo "ERROR: environment prefix is a system or shared root: $env_prefix" >&2
        exit 2
        ;;
esac
case "$env_prefix/" in
    "$project_root/"*) echo "ERROR: environment prefix must not be inside the project checkout" >&2; exit 2 ;;
esac
case "$project_root/" in
    "$env_prefix/"*) echo "ERROR: project checkout must not be inside the environment prefix" >&2; exit 2 ;;
esac
[[ "$(uname -s)" == "Linux" && "$(uname -m)" == "x86_64" ]] || {
    echo "ERROR: the legacy Panda client is gated only for Linux x86_64" >&2
    exit 1
}
openpi_client_package="$project_root/third_party/openpi/packages/openpi-client"
[[ -f "$openpi_client_package/pyproject.toml" ]] || {
    echo "ERROR: pinned OpenPI submodule is missing; run: git submodule update --init third_party/openpi" >&2
    exit 1
}
[[ -f "$project_root/packages/franka-runtime/pyproject.toml" ]] || { echo "ERROR: franka-runtime is missing" >&2; exit 1; }
[[ -f "$REQUIREMENTS" ]] || { echo "ERROR: pinned client requirements are missing" >&2; exit 1; }
python_bin="$env_prefix/bin/python"
[[ -x "$python_bin" ]] || { echo "ERROR: environment Python is missing: $python_bin" >&2; exit 1; }

python_version="$("$python_bin" -c 'import platform; print(platform.python_version())')"
[[ "$python_version" == 3.8.* ]] || { echo "ERROR: expected Python 3.8.x, found $python_version" >&2; exit 1; }
"$python_bin" - <<'PY'
import numpy
import torch

if numpy.__version__ != "1.23.5":
    raise SystemExit(f"expected NumPy 1.23.5, found {numpy.__version__}")
if torch.__version__.split("+", 1)[0] != "1.13.1":
    raise SystemExit(f"expected PyTorch 1.13.1, found {torch.__version__}")
PY

if [[ "$check_only" != true ]]; then
    "$python_bin" -m pip install --requirement "$REQUIREMENTS"
    "$python_bin" -m pip install --no-deps --editable "$openpi_client_package"
    "$python_bin" -m pip install --no-deps --editable "$project_root/packages/franka-runtime"
fi

PROJECT_ROOT="$project_root" "$python_bin" - <<'PY'
import os
from pathlib import Path

import franka_runtime
import openpi_client
import polymetis
import websockets
import zmq

root = Path(os.environ["PROJECT_ROOT"]).resolve()
for name, module in (("openpi_client", openpi_client), ("franka_runtime", franka_runtime)):
    module_path = Path(module.__file__).resolve()
    if root not in module_path.parents:
        raise SystemExit(f"{name} is not loaded from the selected checkout: {module_path}")
if websockets.__version__ != "13.1":
    raise SystemExit(f"expected websockets 13.1, found {websockets.__version__}")
if zmq.__version__ != "25.1.2":
    raise SystemExit(f"expected pyzmq 25.1.2, found {zmq.__version__}")
print("Franka robot-side Python imports passed")
PY
"$python_bin" -m pip check

echo "Robot-side clients verified in ${env_prefix} (Python ${python_version})."
echo "Requirements SHA256: $(sha256sum "$REQUIREMENTS" | awk '{print $1}')"
