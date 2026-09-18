import os
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes


def get_composable_node():

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    return ComposableNode(
        package="trajectory_planner",
        plugin="TrajectoryPlanner",
        name="trajectory_planner",
        parameters=[
            {
                # A static factor to multiply the found steering angle with
                "steering_angle_factor": 1.0,
                # A factor to use the last steering angle for weighted averaging with the current steering angle
                "steering_angle_change_ratio": 1.0,
                # Whether to use the first found point for calculating the steering angle. If false, the average of all points will be used.
                "use_single_point": False,
                # Which point is used when "use_single_point" is set to True
                "use_single_point_at_iteration": 1,
                # The amount of points to use for calculating the average point for the steering angle
                "point_amount_for_average": 3,
                # Use simulation time
                "use_sim_time": use_sim_time,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )


def launch_setup(context: LaunchContext, *args, **kwargs):
    log_level = LaunchConfiguration("log_level").perform(context)
    use_global_composition = (
        LaunchConfiguration("use_global_composition").perform(context).lower() == "true"
    )
    container_name = LaunchConfiguration("container_name").perform(context)

    composabel_node = get_composable_node()

    if use_global_composition:
        setup_action = LoadComposableNodes(
            target_container=container_name,
            composable_node_descriptions=[composabel_node],
        )
    else:
        setup_action = ComposableNodeContainer(
            name="trajectory_planner_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[composabel_node],
            output="screen",
            ros_arguments=["--log-level", ["trajectory_planner:=", log_level]],
        )

    return [setup_action]


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
        default_value="trajectory_planner_container",
        description="Name of the shared container for composable nodes",
    )

    return LaunchDescription(
        [
            log_level_arg,
            use_sim_time_arg,
            use_global_composition_arg,
            container_name_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
