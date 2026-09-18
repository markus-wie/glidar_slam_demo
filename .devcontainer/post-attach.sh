#!/usr/bin/env bash
set -e

echo "[devcontainer] Running post-attach..."

target_bashrc="${HOME}/.bashrc"
if ! grep -qF "source ${WORKSPACE_ROOT}/.devcontainer/setup.bash" "$target_bashrc"; then
    echo "source ${WORKSPACE_ROOT}/.devcontainer/setup.bash" >> "$target_bashrc"
fi

# install dependencies
bash ${WORKSPACE_ROOT}/scripts/install-dependencies.bash

echo "[devcontainer] Setup completed successfully!"
