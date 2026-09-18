#!/bin/bash

set -e

source ${WORKSPACE_ROOT}/install/setup.bash

car_type="${1:-car_damped}"
course_type="${2:-course_short_complex_corridor}"

ros2 launch demo_bringup bringup_sim_launch.py \
	use_sim_time:=true \
	car_type:="${car_type}" \
	course_type:="${course_type}"
