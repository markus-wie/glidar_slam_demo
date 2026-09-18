#!/bin/bash
set -e

build_type="${1:-Release}"
package_name="${2:-}"

# Color codes
BLUE='\033[0;34m'
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${RED}[ATTENTION] Complete build can use up to 20 GB RAM. Ensure 32+ GB RAM + Swap available!${NC}"

# Base CMake args
declare -a cmake_args=(
    "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
    "-DCMAKE_C_COMPILER_LAUNCHER=ccache"
    "-DCMAKE_CXX_COMPILER_LAUNCHER=ccache"
    "--no-warn-unused-cli"
)

case "$build_type" in
    Release)
        echo -e "${BLUE}[INFO] Using Release configuration${NC}"
        cmake_args+=(
            "-DCMAKE_BUILD_TYPE=Release"
            "-DBUILD_TESTING=OFF"
            "-DCMAKE_EXE_LINKER_FLAGS=-fuse-ld=mold"
            "-DCMAKE_SHARED_LINKER_FLAGS=-fuse-ld=mold"
            "-DCMAKE_MODULE_LINKER_FLAGS=-fuse-ld=mold"
            "-DCMAKE_CXX_FLAGS=-Wall -Wextra -Wpedantic"
            "-DCMAKE_C_FLAGS=-Wall -Wextra"   
        )
        build_dir="${WORKSPACE_ROOT}/build/Release"
        ;;
    Debug)
        echo -e "${BLUE}[INFO] Using Debug configuration${NC}"
        cmake_args+=(
            "-DCMAKE_BUILD_TYPE=Debug"
            "-DBUILD_TESTING=ON"
            "-DCMAKE_EXE_LINKER_FLAGS=-fuse-ld=mold --coverage"
            "-DCMAKE_SHARED_LINKER_FLAGS=-fuse-ld=mold --coverage"
            "-DCMAKE_MODULE_LINKER_FLAGS=-fuse-ld=mold --coverage"
            "-DCMAKE_CXX_FLAGS=--coverage -Wall -Wextra -Wpedantic"
            "-DCMAKE_C_FLAGS=--coverage -Wall -Wextra"
        )
        build_dir="${WORKSPACE_ROOT}/build/Debug"
        ;;
    *)
        echo "Invalid build type: '$build_type'. Use: Release or Debug"
        exit 1
        ;;
esac

threads=${COLCON_BUILD_THREADS:-$(nproc --ignore=2)}
echo -e "${BLUE}[INFO] Building with ${threads} parallel threads${NC}"

source "${ROS_INSTALL_PATH}/setup.bash"

if [ -n "$package_name" ]; then
    echo -e "${BLUE}[INFO] Building specific src package: $package_name${NC}"
    colcon build --symlink-install \
        --base-paths src \
        --packages-up-to "$package_name" \
        --cmake-args "${cmake_args[@]}" \
        --parallel-workers "$threads" \
        --build-base "$build_dir"
else
    colcon build --symlink-install \
        --base-paths src \
        --cmake-args "${cmake_args[@]}" \
        --parallel-workers "$threads" \
        --build-base "$build_dir"
fi

echo -e "${GREEN}=== Build completed ✅ ===${NC}"
