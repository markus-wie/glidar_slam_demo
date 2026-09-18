from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


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

    use_global_composition_arg = DeclareLaunchArgument(
        "use_global_composition",
        default_value="false",
        description="Whether to use global composition (if supported)",
    )

    container_name_arg = DeclareLaunchArgument(
        "container_name",
        default_value="center_point_detection_container",
        description="Name of the shared container for composable nodes",
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    bringup_dir = get_package_share_directory("demo_bringup")

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("slam_toolbox"),
                "launch",
                "online_async_launch.py",
            )
        ),
        launch_arguments={
            "slam_params_file": os.path.join(
                bringup_dir, "params", "mapper_params_online_async.yaml"
            ),
            "use_sim_time": use_sim_time,
        }.items(),
    )

    return LaunchDescription(
        [
            log_level_arg,
            use_sim_time_arg,
            use_global_composition_arg,
            container_name_arg,
            slam_toolbox_launch,
        ]
    )
