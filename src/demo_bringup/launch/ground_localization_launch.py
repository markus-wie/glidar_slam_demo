from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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

    return LaunchDescription(
        [
            log_level_arg,
            use_sim_time_arg,
            use_global_composition_arg,
            container_name_arg,
            Node(
                package="sam_ground_slam",
                executable="ground_localization",
                name="ground_localization",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration(
                            "use_sim_time", default="false"
                        ),
                    }
                ],
            ),
        ]
    )
