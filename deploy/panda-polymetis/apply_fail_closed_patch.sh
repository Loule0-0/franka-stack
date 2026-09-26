#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=versions.env
source "$SCRIPT_DIR/versions.env"
PROGRAM_NAME="${0##*/}"
PATCH_FILE="$SCRIPT_DIR/fail_closed.patch"

usage() {
    cat <<EOF
Usage: ${PROGRAM_NAME} --source-root PATH [--check]

Without --check, apply fail_closed.patch to the exact pinned fairo checkout.
The operation is idempotent and refuses unknown tracked modifications.

With --check, require the patch to be already applied exactly. This mode makes
no changes and is used by check_legacy_stack.sh.
EOF
}

source_root="${PANDA_POLYMETIS_SOURCE_ROOT:-}"
check_only=false

while (($#)); do
    case "$1" in
        --source-root) source_root="${2:?missing value for --source-root}"; shift 2 ;;
        --check) check_only=true; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "ERROR: unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

[[ "$source_root" == /* ]] || { echo "ERROR: --source-root must be absolute" >&2; exit 2; }
[[ -d "$source_root/.git" ]] || { echo "ERROR: fairo checkout not found: $source_root" >&2; exit 1; }
[[ -f "$PATCH_FILE" ]] || { echo "ERROR: patch file missing: $PATCH_FILE" >&2; exit 1; }
command -v git >/dev/null 2>&1 || { echo "ERROR: git not found" >&2; exit 3; }

actual_commit="$(git -C "$source_root" rev-parse HEAD)"
[[ "$actual_commit" == "$FAIRO_COMMIT" ]] || {
    echo "ERROR: refusing to patch fairo ${actual_commit}; expected ${FAIRO_COMMIT}" >&2
    exit 1
}

expected_paths=(
    polymetis/polymetis/include/polymetis/clients/franka_hand_client.hpp
    polymetis/polymetis/include/polymetis/clients/franka_panda_client.hpp
    polymetis/polymetis/include/real_time.hpp
    polymetis/polymetis/protos/polymetis.proto
    polymetis/polymetis/python/polymetis/gripper_interface.py
    polymetis/polymetis/python/polymetis/robot_servers/gripper_server.py
    polymetis/polymetis/src/clients/franka_panda_client/franka_hand_client.cpp
    polymetis/polymetis/src/clients/franka_panda_client/franka_panda_client.cpp
    polymetis/polymetis/tests/python/polymetis/test_gripper_interface.py
)

verify_exact_patch_state() {
    git -C "$source_root" apply --reverse --check "$PATCH_FILE" >/dev/null 2>&1 || return 1

    mapfile -t modified_paths < <(git -C "$source_root" diff --name-only --ignore-submodules=all HEAD --)
    ((${#modified_paths[@]} == ${#expected_paths[@]})) || return 1

    declare -A expected_set=()
    local path
    for path in "${expected_paths[@]}"; do
        expected_set["$path"]=1
    done
    for path in "${modified_paths[@]}"; do
        [[ -n "${expected_set[$path]+present}" ]] || return 1
    done

    # Reverse-apply plus the same path set is insufficient: extra hunks in
    # one of the expected files would still pass. Compare the complete canonical
    # Git diff byte-for-byte with the reviewed patch.
    local actual_diff
    actual_diff="$(mktemp)"
    git -C "$source_root" diff --no-ext-diff --ignore-submodules=all HEAD -- "${expected_paths[@]}" >"$actual_diff"
    cmp -s "$actual_diff" "$PATCH_FILE"
    local exact=$?
    rm -f -- "$actual_diff"
    return "$exact"
}

if verify_exact_patch_state; then
    echo "Fail-closed patch is already applied exactly to fairo ${FAIRO_COMMIT}."
    exit 0
fi

if [[ "$check_only" == true ]]; then
    echo "ERROR: fail-closed patch is absent, partial, or mixed with other tracked changes" >&2
    exit 1
fi

git -C "$source_root" diff --quiet --ignore-submodules=all HEAD -- || {
    echo "ERROR: checkout has tracked modifications; refusing to apply the safety patch" >&2
    exit 1
}
git -C "$source_root" apply --check "$PATCH_FILE"
git -C "$source_root" apply "$PATCH_FILE"
verify_exact_patch_state || {
    echo "ERROR: post-apply verification failed" >&2
    exit 1
}

echo "Applied and verified fail-closed patch on fairo ${FAIRO_COMMIT}."
