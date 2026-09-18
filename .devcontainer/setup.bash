#!/bin/bash

if [[ $- == *i* ]]; then
    if [ -f "${ROS_INSTALL_PATH}/setup.bash" ]; then
      source ${ROS_INSTALL_PATH}/setup.bash
      echo "ROS ${ROS_DISTRO} sourced"
    fi

    if [ -f "${WORKSPACE_ROOT}/install/setup.bash" ]; then
      source "${WORKSPACE_ROOT}/install/setup.bash"
      echo "Workspace-Setup sourced"
    fi
  fi
