from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

CAR_TYPES = {"car_damped", "car_rigid", "car_simple"}


def launch_control(context, *args, **kwargs):
    car_type = LaunchConfiguration("car_type").perform(context)
    if car_type not in CAR_TYPES:
        raise RuntimeError(f"Unsupported car_type: {car_type}")

    use_sim_time = (
        LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    )
    demo_bringup_dir = get_package_share_directory("demo_bringup")
    demo_gazebo_dir = get_package_share_directory("demo_gazebo")
    desc_file_path = os.path.join(demo_gazebo_dir, "models", car_type, "model.sdf")
    controller_file_name = (
        "controllers-car-simple.yaml"
        if car_type == "car_simple"
        else "controllers-car-damped.yaml"
    )
    controllers_file = os.path.join(demo_bringup_dir, "params", controller_file_name)

    with open(desc_file_path, "r") as infp:
        robot_desc = infp.read()

    node_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {
                "robot_description": robot_desc,
                "use_sim_time": use_sim_time,
            },
            controllers_file,
        ],
        output="both",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {
                "robot_description": robot_desc,
                "frame_prefix": "",
                "publish_frequency": 100.0,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "30",
            "--service-call-timeout",
            "30",
            "--switch-timeout",
            "30",
        ],
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
            },
        ],
    )

    ackermann_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ackermann_steering_controller",
            "--controller-manager-timeout",
            "30",
            "--service-call-timeout",
            "30",
            "--switch-timeout",
            "30",
        ],
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
            },
        ],
    )

    return [
        # node_controller_manager,
        robot_state_publisher,
        joint_state_broadcaster,
        ackermann_controller,
    ]


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
    
    car_type_arg = DeclareLaunchArgument(
        "car_type",
        default_value="car_damped",
        description="Car model (car_damped|car_rigid|car_simple)",
    )

    return LaunchDescription(
        [
            log_level_arg,
            use_sim_time_arg,
            car_type_arg,
            OpaqueFunction(function=launch_control),
        ]
    )
