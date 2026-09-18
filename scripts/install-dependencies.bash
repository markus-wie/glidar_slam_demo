#!/bin/bash

# When some package defines this as a dependency, a older version of libomp-dev will be installed, that is not compatible with the version of clangd
# So we skip it and install the correct version of libomp-dev in the docker build
skip_keys=("libomp-dev")

# initiliaze rosdep only on the first run
if [[ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]]; then
    # suppress stdout because the hint "please run 'rosdep update' now" is confusing
    # we do this in the next command anyways
    sudo rosdep init >> /dev/null
fi

rosdep update

sudo apt update

rosdep install --from-paths src --ignore-src --skip-keys "${skip_keys[@]}" -r -y
