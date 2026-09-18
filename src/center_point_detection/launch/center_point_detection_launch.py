from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node


def get_composable_node():

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    return ComposableNode(
        package="center_point_detection",
        plugin="CenterPointDetection",
        name="center_point_detection",
        parameters=[
            {
                # In Cam coordinates, the length of the casted rays for lane detection
                "ray_length": 0.7,
                # The ratio of the elliptic shapes of the casted rays for each iteration. 1.0 is a circle, 0.5 is stretched in the x-axis. The first value is for the initial search of the origin, hence it is set to a low value to focus on points close to the car.
                "ray_elliptic_ratios": [0.5, 1.2, 1.6, 1.8, 2.2, 2.2, 2.2, 2.2, 2.2],
                # In Cam coordinates, the width of the lane. Doesn't have to be accurate. Has an influence on where the points are found especially in curves.
                "lane_width": 0.5,
                # The maximum angle between the current search direction and the line segment a intersection got found for.
                # Useful to filter out line segments that are not oriented "parallel" to the course of the road.
                "max_angle": 180.0,
                # In Cam coordinates, the maximum distance the center points can be from the car.
                "max_dist": 1.8,
                # The amount of casted rays for lane detection
                "ray_count": 100,
                # The amount of center points to be found. Also referred to as the iteration count.
                "max_points": 5,
                # To only print the rays on a certain iteration number. 0 to print the rays of all iterations.
                "draw_rays_only_at_iteration_number": 0,
                # Whether to print the casted rays for lane detection on the image
                "draw_rays_in_debug_image": False,
                # Whether to draw the found lane points after translation
                "draw_translated_points": False,
                # Whether to draw the found lane points
                "draw_intersection_points": True,
                # LaneMode: 0 - Both lanes, 1 - Left lane, 2 - Right lane
                "lane_mode": 0,
                # The maximum distance between two points for them to be considered to be reachable from one another.
                # The value should be as small as possible such that points on the same lane are clustered together, and outliers are robustly ommitted.
                "dbscan_eps": 0.12,
                # Amount of points that need to be in the vicinity of dbscan_eps to be considered a "core point"
                "dbscan_min_points": 2,
                # Vehicle parameters (improvable...)
                "vehicle.wheelbase": 0.28,
                "vehicle.wheel_separation": 0.26,
                "vehicle.center_to_cam_offset": 0.62,
                "vehicle.cam_to_world_scale_factor": 2.77,
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
            name="center_point_detection_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[composabel_node],
            output="screen",
            ros_arguments=["--log-level", ["center_point_detection:=", log_level]],
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
        default_value="center_point_detection_container",
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
                name="center_point_detection_debug_throttle",
                package="topic_tools",
                executable="throttle",
                parameters=[
                    {
                        "throttle_type": "messages",
                        "msgs_per_sec": 8.0,
                        "input_topic": "/center_point_detection/debug_image/compressed",
                        "output_topic": "/center_point_detection/debug_image/throttled/compressed",
                        "use_sim_time": use_sim_time,
                    }
                ],
                arguments=[
                    "/center_point_detection/debug_image/compressed",
                ],
            ),
        ]
    )
