import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

CAR_TYPES = {"car_damped", "car_rigid", "car_simple"}
COURSE_TYPES = {
    "course_complex_corridor",
    "course_long",
    "course_small",
    "course_short_complex_corridor",
}


def launch_gazebo(context, *args, **kwargs):
    car_type = LaunchConfiguration("car_type").perform(context)
    course_type = LaunchConfiguration("course_type").perform(context)

    if car_type not in CAR_TYPES:
        raise RuntimeError(f"Unsupported car_type: {car_type}")
    if course_type not in COURSE_TYPES:
        raise RuntimeError(f"Unsupported course_type: {course_type}")

    demo_gazebo_dir = get_package_share_directory("demo_gazebo")
    world_sdf_file = os.path.join(demo_gazebo_dir, "worlds", "world.sdf")
    world_sdf = xacro.process_file(
        world_sdf_file,
        mappings={"car_type": car_type, "course_type": course_type},
    )
    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".sdf", prefix="demo_gazebo_", delete=False
    ) as world_sdf_file_handle:
        world_sdf_file_handle.write(getattr(world_sdf, "toxml")())
        generated_world_sdf_file = world_sdf_file_handle.name

    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Mockup for ros_gz_sim.launch.py under kilted
    ros_gz_sim_mockup_dir = get_package_share_directory("demo_gazebo")

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [
                "-r -g -v 4",
                " --gui-config ",
                os.path.join(
                    demo_gazebo_dir,
                    "config",
                    "gz_gui.config",
                ),
            ],
            "lockstep": "true",
            "on_exit_shutdown": "true",
        }.items(),
    )

    gz_sim_server_and_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_mockup_dir, "launch", "ros_gz_sim.launch.py")
        ),
        launch_arguments={
            "use_composition": "true",
            "bridge_name": "gz_bridge",
            "config_file": os.path.join(demo_gazebo_dir, "config", "bridge.yaml"),
            # by not overriding container_name, the argument set by a top level launch file is used.
            # set create_own_container to false to attach to the container launched by the top level launch file instead of creating a new one.
            # "container_name": "ros_gz_container",
            "create_own_container": "false",
            "bridge_log_level": "info",
            "world_sdf_file": generated_world_sdf_file,
        }.items(),
    )

    return [gz_sim_server_and_bridge, gz_sim_gui]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "car_type",
                default_value="car_damped",
                description="Car model (car_damped|car_rigid|car_simple)",
            ),
            DeclareLaunchArgument(
                "course_type",
                default_value="course_short_complex_corridor",
                description="Course model (course_complex_corridor|course_long|course_small|course_short_complex_corridor)",
            ),
            OpaqueFunction(function=launch_gazebo),
        ]
    )
