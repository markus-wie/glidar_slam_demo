import os

from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node


def get_composable_node():

    roi_top = 0.217
    roi_bottom = 0.054
    roi_right = 0.1875
    roi_left = 0.1875
    roll = 0.0
    pitch = 81.5
    yaw = 0.0
    adaptive_threshold_block_size = 91
    adaptive_threshold_c = -50.0

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    return ComposableNode(
        package="image_processing",
        plugin="image_processing::ImageProcessingNode",
        name="image_processing",
        parameters=[
            {
                # Height of the ROI in pixels
                "ROI_Top": roi_top,
                "ROI_Bottom": roi_bottom,
                "ROI_Right": roi_right,
                "ROI_Left": roi_left,
                # Epsilon for the chain approximation algorithm. Higher values will result in
                # fewer points in the contours. Used to reduce the amount of segments resulting
                # in less computational effort in the center_point_detection.
                "chain_approx_epsilon": 0.8,
                # Whether to apply the contours optimization step
                "apply_contours_optimization": True,
                # The size of the kernel used for calculating the adaptive threshold.
                "adaptive_threshold_block_size": adaptive_threshold_block_size,
                # The constant subtracted from the mean or weighted mean in the adaptive threshold.
                "adaptive_threshold_c": adaptive_threshold_c,
                # The rotational components of the camera plane relative to the ground plane.
                # Used to calculate a homography matrix for the bird's eye view transformation.
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
                # The BEV transformation is somehow "zoomed in". We therefore "zoom out" by this factor.
                "bev_scale_factor": 0.1,
                "print_timings": False,
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
            name="image_processing_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[composabel_node],
            output="screen",
            ros_arguments=["--log-level", ["image_processing:=", log_level]],
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
        default_value="image_processing_container",
        description="Name of the shared container for composable nodes",
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    return LaunchDescription(
        [
            log_level_arg,
            use_sim_time_arg,
            use_global_composition_arg,
            container_name_arg,
            OpaqueFunction(function=launch_setup),
            Node(
                name="line_segment_detection_debug_throttle",
                package="topic_tools",
                executable="throttle",
                parameters=[
                    {
                        "throttle_type": "messages",
                        "msgs_per_sec": 8.0,
                        "input_topic": "/line_segment_detection/detected_lines/debug_image/compressed",
                        "output_topic": "/line_segment_detection/detected_lines/debug_image/throttled/compressed",
                        "use_sim_time": use_sim_time,
                    }
                ],
                arguments=[
                    "/line_segment_detection/detected_lines/debug_image/compressed",
                ],
            ),
            Node(
                name="line_segment_detection_debug_throttle_2",
                package="topic_tools",
                executable="throttle",
                parameters=[
                    {
                        "throttle_type": "messages",
                        "msgs_per_sec": 8.0,
                        "input_topic": "/line_segment_detection/warped_image/debug_image/compressed",
                        "output_topic": "/line_segment_detection/warped_image/debug_image/throttled/compressed",
                        "use_sim_time": use_sim_time,
                    }
                ],
                arguments=[
                    "/line_segment_detection/warped_image/debug_image/compressed",
                ],
            ),
        ]
    )
