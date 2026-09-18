<p align="center">
  <h1 align="center">GLiDAR-SLAM Demo</h1>
  <p align="center">
    <strong>Markus Wiechers</strong>
  </p>
</p>
<p align="center">
  <a href="">
    <img src="./media/gazebo_ortho_view.gif" width="80%">
  </a>
</p>


Provides a complete ros2 development and gazebo simulation environment featuring simple pure pursuit vehicle control for small scale autonomous racing experiments.
The system utilizes ros2_control, tf2, ekf sensor fusion, and diverse SLAM setups, especially the GLiDAR SLAM system, but also integrates slam_toolbox and rtabmap.

## Prerequisites

- Docker Engine for the containerized workflow.
- For the VS Code workflow: VS Code with the **Dev Containers** extension.
- A graphical desktop session for Gazebo, rviz2 and rqt visualizations.

## VS Code Setup

1. Open this repository in VS Code.
2. Run **Dev Containers: Reopen in Container**. Choose the Linux or Windows(WSL2) configuration for the host.
3. The container's post-attach step installs ROS dependencies automatically. Wait for it to finish.
4. Run **Tasks: Run Task** and select **ROS: Build Package**. Choose `Release` and leave the package selection empty to build the workspace.
5. Run **Tasks: Run Task** and select **ROS: Start Launch-File**. Select the requested car and course from the prompts.

The launch task offers these modes:

| Selection | Values |
| --- | --- |
| Car type | `car_damped`, `car_rigid`, `car_simple` |
| Course type | `course_complex_corridor`, `course_long`, `course_small`, `course_short_complex_corridor` |

`car_damped` features a full 4-bar linkage suspension system for all wheels, providing more realistic vehicle motion. In this mode the simulation runs with the `pgs` solver and a 0.001 s step size.

## Manual Docker Setup

On a Linux host, build the image from the repository root:

```bash
docker build \
	-f .devcontainer/Dockerfile \
	--build-arg ROS_DISTRO=kilted \
	--build-arg USERNAME=ros \
	--build-arg USER_UID="$(id -u)" \
	--build-arg USER_GID="$(id -g)" \
	-t glidar-slam-demo .
```

Permit local containers to use a X server, then start an interactive workspace container:

```bash
xhost +local:docker

docker run --rm -it \
	--network host \
	--ipc host \
	--privileged \
	--env DISPLAY \
	--env XAUTHORITY=/home/ros/.Xauthority \
	--volume /tmp/.X11-unix:/tmp/.X11-unix \
	--volume "$HOME/.Xauthority:/home/ros/.Xauthority:ro" \
	--volume "$PWD:/workspaces/glidar_slam_demo" \
	--workdir /workspaces/glidar_slam_demo \
	glidar-slam-demo bash
```

Inside the container, set the workspace variables and install dependencies:

```bash
export WORKSPACE_ROOT="$PWD"
export ROS_INSTALL_PATH=/opt/ros/kilted
./scripts/install-dependencies.bash
```

## Build

From the workspace root, build all packages in Release mode:

```bash
./scripts/build.bash Release
```

## Run Modes

After a successful build, source the workspace and launch the simulation with explicit selections:

```bash
source install/setup.bash
ros2 launch demo_bringup bringup_sim_launch.py \
	use_sim_time:=true \
	car_type:=car_rigid \
	course_type:=course_short_complex_corridor
```

Or use the wrapper, whose positional arguments are `car_type` followed by `course_type`:

```bash
./scripts/launch.bash car_damped course_short_complex_corridor
./scripts/launch.bash car_simple course_small
```

There should be three windows open: Gazebo, rviz2, and rqt.

## Driving the vehicle

In rqt Parameter Reconfigure, select the `mission_controller` and enable the boolean parameter `mission.racing`. Now, the vehicle should automatically start driving.

Alternatively, you can set the parameter value from the terminal:

```bash
ros2 param set /mission_controller mission.racing true
```

and to disable it:

```bash
ros2 param set /mission_controller mission.racing false
```
