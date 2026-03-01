#!/bin/bash
#
# Syncs submodule commits to match the versions pinned in MuJoCo's
# cmake/MujocoDependencies.cmake. Run after updating the mujoco submodule.

set -euo pipefail

readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly NC='\033[0m'

error() {
    echo -e "[${RED}error${NC}]: ${1}" >&2
    exit 1
}

warning() {
    echo -e "[${YELLOW}warning${NC}]: ${1}" >&2
}

info() {
    echo -e "[${GREEN}info${NC}]: ${1}" >&2
}

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
CMAKE_FILE="$SCRIPT_DIR/mujoco/cmake/MujocoDependencies.cmake"

if [[ ! -f "$CMAKE_FILE" ]]; then
    error "$CMAKE_FILE not found"
fi

# Map CMake dependency names to submodule directory names.
# Only entries where the names differ need to be listed here.
declare -A NAME_MAP=(
    [ccd]=libccd
)

# CMake deps that are test/build-only and intentionally not submodules.
declare -A SKIP_DEPS=(
    [Eigen3]=1
    [abseil]=1
    [gtest]=1
    [benchmark]=1
)

# Extract MUJOCO_DEP_VERSION_<name> <hash> pairs from the CMake file.
# The version name and commit hash are on separate lines, e.g.:
#   set(MUJOCO_DEP_VERSION_lodepng
#       17d08dd26cac4d63f43af217ebd70318bfb8189c
parse_cmake_versions() {
    awk '
        /MUJOCO_DEP_VERSION_/ {
            match($0, /MUJOCO_DEP_VERSION_([A-Za-z0-9_]+)/, m)
            name = m[1]
            next
        }
        name && /^[[:space:]]*[0-9a-f]{40}/ {
            match($0, /([0-9a-f]{40})/, m)
            print name, m[1]
            name = ""
        }
    ' "$CMAKE_FILE"
}

updated=0
skipped=0
errors=0

while read -r cmake_name commit; do
    if [[ -n "${SKIP_DEPS[$cmake_name]:-}" ]]; then
        continue
    fi

    # Resolve the submodule directory name
    dir_name="${NAME_MAP[$cmake_name]:-$cmake_name}"
    submodule_path="$SCRIPT_DIR/$dir_name"

    if [[ ! -d "$submodule_path/.git" && ! -f "$submodule_path/.git" ]]; then
        warning "$dir_name: no submodule found, skipping"
        skipped=$((skipped + 1))
        continue
    fi

    current=$(git -C "$submodule_path" rev-parse HEAD)

    if [[ "$current" == "$commit" ]]; then
        info "$dir_name is up to date (${commit:0:12})"
        continue
    fi

    warning "$dir_name: ${current:0:12} -> ${commit:0:12}"

    # Fetch in case the target commit isn't available locally yet
    if ! git -C "$submodule_path" cat-file -e "$commit" 2>/dev/null; then
        info "$dir_name: fetching from origin..."
        git -C "$submodule_path" fetch origin
    fi

    if git -C "$submodule_path" checkout "$commit" 2>/dev/null; then
        info "$dir_name: updated"
        updated=$((updated + 1))
    else
        error "failed to checkout $commit in $dir_name"
    fi
done < <(parse_cmake_versions)

echo ""
info "done: $updated updated, $skipped skipped (no submodule), $errors errors"
