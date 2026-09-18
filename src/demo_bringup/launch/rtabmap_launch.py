import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
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
        default_value="true",
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

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    bringup_dir = get_package_share_directory("demo_bringup")

    parameters = [
        {
            "use_sim_time": use_sim_time,
            "frame_id": "base_link",
            "subscribe_rgbd": True,
            "subscribe_depth": False,
            "subscribe_scan": True,
            "approx_sync": True,
            "approx_sync_max_interval": 0.05,
            "queue_size": 20,
            "topic_queue_size": 20,
            "sync_queue_size": 20,
            "Rtabmap/DetectionRate": "10.0",
            "Reg/Strategy": "2",  # 0=Visual, 1=ICP, 2=Visual+ICP
            # "GTSAM/Optimizer": "0",  # 0=Levenberg 1=GaussNewton 2=Dogleg
            "RGBD/NeighborLinkRefining": "true",
            "Reg/Force3DoF": "true",  # x,y,yaw only, rest is 0
            "Optimizer/Slam2D": "true",
            "RGBD/ProximityBySpace": "true",
            "RGBD/LoopClosureReextractFeatures": "true",
            "RGBD/LocalBundleOnLoopClosure": "true",
            "RGBD/OptimizeMaxError": "3.0",
            "RGBD/LinearUpdate": "0.05",
            "RGBD/AngularUpdate": "0.1",
            # "Icp/VoxelSize": "0.05",
            # "Icp/MaxTranslation": "0.5",
            "Optimizer/Strategy": "2",
            "Kp/RoiRatios": "0.0 0.0 0.0 0.25",  # cut the bottom x% of the image to avoid detecting keypoints on the robot itself. Must be set and found manually
            "Kp/MaxFeatures": "1000",  # default is 500
            # "Vis/RoiRatios": "0.0 0.0 0.0 0.4",
            # "Grid/DepthRoiRatios": "0.0 0.0 0.0 0.4",
            # "Grid/3D": "true",
            # "Grid/Sensor": "1",
            # "Grid/GroundIsObstacle": "true",
            # "Grid/RangeMin": "0.3",
            "Mem/DepthCompressionFormat": ".png",  # to suppress a warning
            "Mem/ReduceGraph": "false",
            # "Mem/RehearsalSimilarity": "0.8",
            # "Mem/NotLinkedNodesKept": "false",
        }
    ]

    return LaunchDescription(
        [
            use_sim_time_arg,
            Node(
                package="rtabmap_sync",
                executable="rgbd_sync",
                name="rgbd_sync",
                output="screen",
                parameters=parameters,
                remappings=[
                    ("rgb/image", "/camera/color/image_raw"),
                    ("depth/image", "/camera/aligned_depth/image_raw"),
                    ("rgb/camera_info", "/camera/color/camera_info"),
                    ("rgbd_image", "rgbd_image"),
                ],
                namespace="rtabmap",
            ),
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                name="rtabmap",
                output="screen",
                parameters=parameters,
                remappings=[
                    ("rgbd_image", "rgbd_image"),
                    ("scan", "/scan"),
                    ("odom", "/odometry/filtered"),
                ],
                arguments=["-d"],  # Warning: deletes previous database
                namespace="rtabmap",
            ),
            # Node(
            #     package="rtabmap_viz",
            #     executable="rtabmap_viz",
            #     name="rtabmap_viz",
            #     output="screen",
            #     parameters=parameters,
            #     remappings=[
            #         # ("odom", "/odometry/filtered"),
            #         ("rgbd_image", "rgbd_image"),
            #     ],
            #     namespace="rtabmap",
            # ),
            Node(
                package="sam_ground_slam",
                executable="rtabmap_ground_mapping",
                name="rtabmap_ground_mapping",
                output="screen",
                namespace="rtabmap",
            ),
        ]
    )
