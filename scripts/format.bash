#!/bin/bash

set -e

BLUE='\033[0;34m'
GREEN='\033[0;32m'
NC='\033[0m'

CLANG_FORMAT_CONFIG="${WORKSPACE_ROOT}/.clang-format"

find_excluded_files() {
    local root="$1"
    local name="$2"
    local find_args=("$root")

    find_args+=(
        -type d \( -name .git -o -name .venv -o -name __pycache__ \
        -o -name build -o -name install -o -name log -o -name vendor \
        -o -name node_modules \) -prune -o
    )

    find_args+=(-type f -name "$name" -print)
    find "${find_args[@]}"
}

if [[ -n "${1:-}" ]]; then
    target_dir="$1"
    target_msg="$1"

    if [[ "$(basename "$target_dir")" == "package.xml" ]]; then
        target_dir="$(dirname "$target_dir")"
    fi
else
    target_dir="${WORKSPACE_ROOT}/src"
    target_msg="all packages under ${WORKSPACE_ROOT}/src"
fi

echo -e "${BLUE}[INFO] Formatting files for ${target_msg}...${NC}"

# Gather targets
readarray -t source_files < <(for ext in c cc cpp cxx h hh hpp hxx; do find_excluded_files "$target_dir" "*.$ext"; done)
cmake_files=$(find_excluded_files "$target_dir" "CMakeLists.txt")

if (( ${#source_files[@]} == 0 )); then
    echo "No C/C++ source files found under ${target_dir}." >&2
    exit 1
fi

ament_clang_format "${source_files[@]}" --reformat --config "${CLANG_FORMAT_CONFIG}"

echo -e "${GREEN}[OK] Formatting complete for ${target_msg}.${NC}"
