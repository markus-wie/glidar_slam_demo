import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer


def generate_launch_description():

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=["INFO"],
        description="Logging level",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Enable ros2 simulation time",
    )

    use_global_composition_arg = DeclareLaunchArgument(
        "use_global_composition",
        default_value="true",
        description="Whether to use global composition (if supported)",
    )

    container_name_arg = DeclareLaunchArgument(
        "container_name",
        default_value="demo_container",
        description="Name of the shared container for composable nodes",
    )

    scenario_arg = DeclareLaunchArgument(
        "scenario",
        default_value="racing",
        description="Scenario key (racing|parking|overtaking|pursuit) -> selects a world file",
    )

    car_type_arg = DeclareLaunchArgument(
        "car_type",
        default_value="car_damped",
        description="Car model (car_damped|car_rigid|car_simple)",
    )

    course_type_arg = DeclareLaunchArgument(
        "course_type",
        default_value="course_short_complex_corridor",
        description="Course model (course_complex_corridor|course_long|course_small|course_short_complex_corridor)",
    )

    camera_pitch_offset_arg = DeclareLaunchArgument(
        "camera_pitch_offset_deg",
        default_value="0.0",
        description="Pitch offset in degrees added to the base_link -> camera transform "
        "of the robot description only (positive = pitch down). The simulated camera is unchanged.",
    )

    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Start the Gazebo GUI, RViz and RQT",
    )

    racing_arg = DeclareLaunchArgument(
        "racing",
        default_value="false",
        description="Enable lane following at startup (mission.racing)",
    )

    log_level = LaunchConfiguration("log_level")

    container = ComposableNodeContainer(
        name="demo_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[],
        output="screen",
        ros_arguments=["--log-level", log_level],
    )

    bringup_dir = get_package_share_directory("demo_bringup")
    glidar_slam_ros_dir = get_package_share_directory("glidar_slam_ros")
    demo_gazebo_dir = get_package_share_directory("demo_gazebo")
    image_processing_dir = get_package_share_directory("image_processing")
    center_point_detection_dir = get_package_share_directory("center_point_detection")
    trajectory_planner_dir = get_package_share_directory("trajectory_planner")

    use_sim_time = LaunchConfiguration("use_sim_time", default="False")
    car_type = LaunchConfiguration("car_type")
    course_type = LaunchConfiguration("course_type")

    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("gui")),
        arguments=["-d", os.path.join(bringup_dir, "config", "demo.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # RQT
    rqt = Node(
        package="rqt_gui",
        executable="rqt_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("gui")),
        arguments=[
            "--clear-config",
            "--perspective-file",
            os.path.join(bringup_dir, "config", "demo.perspective"),
            "--force-discover",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    delayed_gui_launch = TimerAction(
        period=5.0,
        actions=[rviz, rqt],
    )

    # multiplexer for different cmd_vel sources
    # The ackermann controller expects instructions on the "/ackermann_steering_controller/reference" topic which is remapped to the output of the multiplexer.
    # interfaces with the system to allow mission control via parameter callbacks
    # multiplexer for different cmd_vel sources
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[
            os.path.join(bringup_dir, "params", "twist_mux.yaml"),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[("/cmd_vel_out", "/ackermann_steering_controller/reference")],
    )

    # Launch files
    sim_launch_file = os.path.join(demo_gazebo_dir, "launch", "demo_gazebo.launch.py")
    lidar_slam_launch_file = os.path.join(bringup_dir, "launch", "lidar_slam_launch.py")
    control_launch_file = os.path.join(bringup_dir, "launch", "control_launch.py")
    navigation_launch_file = os.path.join(bringup_dir, "launch", "nav2_launch.py")
    localization_launch_file = os.path.join(
        bringup_dir, "launch", "nav2_lidar_amcl_launch.py"
    )
    ekf_launch_file = os.path.join(bringup_dir, "launch", "ekf_launch.py")
    glidar_slam_launch_file = os.path.join(
        glidar_slam_ros_dir, "launch", "glidar_slam.launch.py"
    )
    rtabmap_launch_file = os.path.join(bringup_dir, "launch", "rtabmap_launch.py")
    ground_localization_launch_file = os.path.join(
        bringup_dir, "launch", "ground_localization_launch.py"
    )

    nav2_params_file = os.path.join(bringup_dir, "params", "nav2_params.yaml")

    delayed_rtabmap_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rtabmap_launch_file),
            ),
        ],
    )

    mission_controller = Node(
        package="mission_controller",
        executable="mission_controller",
        name="mission_controller",
        output="screen",
        parameters=[
            {
                "mission.racing": ParameterValue(
                    LaunchConfiguration("racing"), value_type=bool
                )
            }
        ],
    )
    image_processing_launch_file = os.path.join(
        image_processing_dir, "launch", "image_processing_launch.py"
    )
    center_point_detection_launch_file = os.path.join(
        center_point_detection_dir, "launch", "center_point_detection_launch.py"
    )
    trajectory_planner_launch_file = os.path.join(
        trajectory_planner_dir, "launch", "trajectory_planner_launch.py"
    )

    return LaunchDescription(
        [
            log_level_arg,
            use_sim_time_arg,
            use_global_composition_arg,
            container_name_arg,
            scenario_arg,
            car_type_arg,
            course_type_arg,
            camera_pitch_offset_arg,
            gui_arg,
            racing_arg,
            container,
            mission_controller,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(image_processing_launch_file),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(center_point_detection_launch_file),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(trajectory_planner_launch_file),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_launch_file),
            ),
            delayed_gui_launch,
            twist_mux_node,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(control_launch_file),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ekf_launch_file),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(glidar_slam_launch_file),
            ),
            # delayed_rtabmap_launch,
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(lidar_slam_launch_file),
            # ),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(ground_localization_launch_file),
            # ),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(rtabmap_launch_file),
            # ),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(navigation_launch_file),
            #     launch_arguments={
            #         "params_file": nav2_params_file,
            #     }.items(),
            # ),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(localization_launch_file),
            #     launch_arguments={
            #         "params_file": nav2_params_file,
            #         "map": "/workspaces/glidar_slam_demo/glidar_map2.yaml",
            #     }.items(),
            # ),
        ]
    )
