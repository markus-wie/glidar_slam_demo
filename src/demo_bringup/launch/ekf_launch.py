from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=["INFO"],
        description="Logging level",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Enable ros2 simulation time",
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    bringup_dir = get_package_share_directory("demo_bringup")

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(bringup_dir, "params", "ekf.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            log_level_arg,
            use_sim_time_arg,
            robot_localization_node,
        ]
    )
