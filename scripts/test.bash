#!/bin/bash
set -e

package_name="${1:-}"

BLUE='\033[0;34m'
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'

source "${WORKSPACE_ROOT}/install/setup.bash"

colcon_args=(
    --base-paths src
    --event-handlers console_direct+
)

if [ -n "$package_name" ]; then
    input_name=$(basename "$package_name")
    if [ "$input_name" = "package.xml" ]; then
        input_name=$(basename "$(dirname "$package_name")")
    fi

    read -r real_pkg_name pkg_path <<< "$(colcon list --base-paths src | awk -v pkg="$input_name" '
        tolower($1) == tolower(pkg) || tolower($2) ~ "/" tolower(pkg) "$" {print $1, $2; exit}
    ')"

    if [ -z "$real_pkg_name" ] || [[ "$pkg_path" != src/* ]]; then
        echo -e "${RED}[ERROR] Could not find a src package matching '$package_name'.${NC}"
        exit 1
    fi

    package_name="$real_pkg_name"
    colcon_args+=(--packages-select "$package_name")
    target_msg="package '$package_name'"
else
    target_msg="all packages under ${WORKSPACE_ROOT}/src"
fi

echo -e "${BLUE}[INFO] Testing ${target_msg}...${NC}"
colcon test "${colcon_args[@]}"
colcon test-result --verbose

echo -e "${GREEN}[OK] Tests complete for ${target_msg}.${NC}"
