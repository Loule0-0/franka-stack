#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=versions.env
source "$SCRIPT_DIR/versions.env"
PROGRAM_NAME="${0##*/}"

usage() {
    cat <<EOF
Usage: ${PROGRAM_NAME} --source-root PATH --env-prefix PATH \\
  --robot-system-version VERSION --robot-server-version N \\
  --gripper-server-version N

Strictly compare a local historical Panda/Polymetis installation and the
versions read from Desk against versions.env. This is a version audit only;
it does not establish realtime correctness or safe physical behavior.
EOF
}

source_root="${PANDA_POLYMETIS_SOURCE_ROOT:-}"
env_prefix="${PANDA_POLYMETIS_ENV_PREFIX:-}"
robot_system_version=""
robot_server_version=""
gripper_server_version=""

while (($#)); do
    case "$1" in
        --source-root) source_root="${2:?missing value for --source-root}"; shift 2 ;;
        --env-prefix) env_prefix="${2:?missing value for --env-prefix}"; shift 2 ;;
        --robot-system-version) robot_system_version="${2:?missing value for --robot-system-version}"; shift 2 ;;
        --robot-server-version) robot_server_version="${2:?missing value for --robot-server-version}"; shift 2 ;;
        --gripper-server-version) gripper_server_version="${2:?missing value for --gripper-server-version}"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "ERROR: unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

[[ "$source_root" == /* && "$env_prefix" == /* ]] || { echo "ERROR: source and environment paths must be absolute" >&2; exit 2; }
[[ -n "$robot_system_version" && -n "$robot_server_version" && -n "$gripper_server_version" ]] || {
    echo "ERROR: enter all three versions exactly as shown in Franka Desk" >&2
    exit 2
}
command -v git >/dev/null 2>&1 || { echo "ERROR: git not found" >&2; exit 3; }
command -v sha256sum >/dev/null 2>&1 || { echo "ERROR: sha256sum not found" >&2; exit 3; }

expected_torch_version="1.13.1"
expected_numpy_version="1.23.5"

failures=0
check_equal() {
    local label="$1"
    local actual="$2"
    local expected="$3"
    if [[ "$actual" == "$expected" ]]; then
        printf '[PASS] %s: %s\n' "$label" "$actual"
    else
        printf '[FAIL] %s: got %s, expected %s\n' "$label" "$actual" "$expected" >&2
        failures=$((failures + 1))
    fi
}

if [[ ! -d "$source_root/.git" ]]; then
    echo "[FAIL] fairo checkout not found: $source_root" >&2
    failures=$((failures + 1))
else
    check_equal "fairo commit" "$(git -C "$source_root" rev-parse HEAD)" "$FAIRO_COMMIT"
    if bash "$SCRIPT_DIR/apply_fail_closed_patch.sh" --source-root "$source_root" --check; then
        echo "[PASS] exact fail-closed patch is applied"
    else
        echo "[FAIL] exact fail-closed patch is not applied" >&2
        failures=$((failures + 1))
    fi

    libfranka_path="$source_root/polymetis/polymetis/src/clients/franka_panda_client/third_party/libfranka"
    if [[ -d "$libfranka_path/.git" || -f "$libfranka_path/.git" ]]; then
        check_equal "libfranka commit" "$(git -C "$libfranka_path" rev-parse HEAD)" "$LIBFRANKA_COMMIT"
    else
        echo "[FAIL] libfranka submodule is not initialized" >&2
        failures=$((failures + 1))
    fi
fi

python_version=""
torch_version=""
numpy_version=""
if [[ -x "$env_prefix/bin/python" ]]; then
    python_version="$("$env_prefix/bin/python" -c 'import platform; print(platform.python_version())')"
    [[ "$python_version" == "${POLYMETIS_PYTHON_VERSION}."* ]] && echo "[PASS] Python: $python_version" || {
        echo "[FAIL] Python: ${python_version}, expected ${POLYMETIS_PYTHON_VERSION}.x" >&2
        failures=$((failures + 1))
    }
    if torch_version="$("$env_prefix/bin/python" -c 'import torch; print(torch.__version__.split("+", 1)[0])' 2>/dev/null)"; then
        check_equal "PyTorch" "$torch_version" "$expected_torch_version"
    else
        echo "[FAIL] PyTorch import/version check" >&2
        failures=$((failures + 1))
    fi
    if numpy_version="$("$env_prefix/bin/python" -c 'import numpy; print(numpy.__version__)' 2>/dev/null)"; then
        check_equal "NumPy" "$numpy_version" "$expected_numpy_version"
    else
        echo "[FAIL] NumPy import/version check" >&2
        failures=$((failures + 1))
    fi
    if "$env_prefix/bin/python" -c 'import polymetis' >/dev/null 2>&1; then
        echo "[PASS] polymetis Python import"
    else
        echo "[FAIL] polymetis Python import" >&2
        failures=$((failures + 1))
    fi
    if "$env_prefix/bin/python" -m pip check; then
        echo "[PASS] Python dependency consistency (pip check)"
    else
        echo "[FAIL] Python dependency consistency (pip check)" >&2
        failures=$((failures + 1))
    fi
else
    echo "[FAIL] environment Python not found: $env_prefix/bin/python" >&2
    failures=$((failures + 1))
fi

arm_binary="$env_prefix/bin/franka_panda_client"
hand_binary="$env_prefix/bin/franka_hand_client"
if [[ -x "$arm_binary" && -x "$hand_binary" ]]; then
    echo "[PASS] franka_panda_client and franka_hand_client executables are installed"
    provenance_file="$source_root/.franka_real_pi05_build_provenance"
    if [[ -r "$provenance_file" ]]; then
        provenance_value() {
            local key="$1"
            awk -F= -v key="$key" '$1 == key {sub(/^[^=]*=/, ""); print; exit}' "$provenance_file"
        }
        expected_patch_sha="$(sha256sum "$SCRIPT_DIR/fail_closed.patch" | awk '{print $1}')"
        environment_file="$source_root/polymetis/polymetis/environment.yml"
        actual_arm_binary_sha="$(sha256sum "$arm_binary" | awk '{print $1}')"
        actual_hand_binary_sha="$(sha256sum "$hand_binary" | awk '{print $1}')"
        check_equal "build provenance fairo commit" "$(provenance_value FAIRO_COMMIT)" "$FAIRO_COMMIT"
        check_equal "build provenance libfranka commit" "$(provenance_value LIBFRANKA_COMMIT)" "$LIBFRANKA_COMMIT"
        check_equal "build provenance patch hash" "$(provenance_value PATCH_SHA256)" "$expected_patch_sha"
        if [[ -f "$environment_file" ]]; then
            expected_environment_sha="$(sha256sum "$environment_file" | awk '{print $1}')"
            check_equal "build provenance environment hash" "$(provenance_value ENVIRONMENT_SHA256)" "$expected_environment_sha"
        else
            echo "[FAIL] pinned environment file is missing: $environment_file" >&2
            failures=$((failures + 1))
        fi
        check_equal "build provenance Python version" "$(provenance_value PYTHON_VERSION)" "$python_version"
        check_equal "build provenance PyTorch version" "$(provenance_value PYTORCH_VERSION)" "$torch_version"
        check_equal "build provenance NumPy version" "$(provenance_value NUMPY_VERSION)" "$numpy_version"
        check_equal "build provenance arm binary path" "$(provenance_value ARM_BINARY_PATH)" "$arm_binary"
        check_equal "installed arm binary hash" "$actual_arm_binary_sha" "$(provenance_value ARM_BINARY_SHA256)"
        check_equal "build provenance hand binary path" "$(provenance_value HAND_BINARY_PATH)" "$hand_binary"
        check_equal "installed hand binary hash" "$actual_hand_binary_sha" "$(provenance_value HAND_BINARY_SHA256)"
    else
        echo "[FAIL] build provenance is missing: $provenance_file" >&2
        failures=$((failures + 1))
    fi
else
    echo "[FAIL] arm or hand client executable is missing" >&2
    failures=$((failures + 1))
fi

check_equal "Robot System" "$robot_system_version" "$EXPECTED_ROBOT_SYSTEM_VERSION"
check_equal "robot server" "$robot_server_version" "$EXPECTED_ROBOT_SERVER_VERSION"
check_equal "gripper server" "$gripper_server_version" "$EXPECTED_GRIPPER_SERVER_VERSION"

echo "[INFO] status: ${PANDA_STACK_STATUS}"
echo "[INFO] passing this audit does not replace a realtime audit or staged hardware test"
if ((failures > 0)); then
    echo "[SUMMARY] legacy stack audit failed: ${failures} issue(s)" >&2
    exit 1
fi
echo "[SUMMARY] pinned-version audit passed"
