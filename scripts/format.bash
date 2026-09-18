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

    while IFS= read -r submodule_path; do
        [ -n "$submodule_path" ] || continue
        find_args+=(-path "${WORKSPACE_ROOT}/${submodule_path}" -prune -o)
    done < <(awk '$1 == "path" && $2 == "=" { print $3 }' "${WORKSPACE_ROOT}/.gitmodules")

    find_args+=(-type f -name "$name" -print)
    find "${find_args[@]}"
}

target_dir="${1:-${WORKSPACE_ROOT}/src}"
target_msg="${1:-all packages under ${WORKSPACE_ROOT}/src}"

echo -e "${BLUE}[INFO] Formatting files for ${target_msg}...${NC}"

# Gather targets
source_files=$(for ext in c cc cpp cxx h hh hpp hxx; do find_excluded_files "$target_dir" "*.$ext"; done)
cmake_files=$(find_excluded_files "$target_dir" "CMakeLists.txt")

ament_clang_format $source_files --reformat --config "${CLANG_FORMAT_CONFIG}"

echo -e "${GREEN}[OK] Formatting complete for ${target_msg}.${NC}"
