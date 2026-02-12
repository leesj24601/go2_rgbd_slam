from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    camera_remappings = [
        ("rgb/image", "/camera/color/image_raw"),
        ("depth/image", "/camera/depth/image_rect_raw"),
        ("rgb/camera_info", "/camera/camera_info"),
    ]

    # Static TF 1: base_link → camera_link (위치만, 회전 없음)
    base_to_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0.30", "0.0", "0.05",
            "0", "0", "0",
            "base_link",
            "camera_link",
        ],
    )

    # Static TF 2: camera_link → camera_optical_frame (회전만, 위치 없음)
    camera_to_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0", "0", "0",
            "-1.5708", "0", "-1.5708",
            "camera_link",
            "camera_optical_frame",
        ],
    )

    rtabmap_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        output="screen",
        parameters=[
            {
                "frame_id": "camera_link",
                "map_frame_id": "map",
                "odom_frame_id": "odom",
                "subscribe_depth": True,
                "subscribe_odom_info": False,
                "approx_sync": True,
                "approx_sync_max_interval": 0.5,
                "publish_tf": True,
                "tf_delay": 0.05,
                "wait_for_transform": 0.5,
                "qos": 1,
                "queue_size": 5,
                "use_sim_time": use_sim_time,
                # IMU 구독
                "subscribe_imu": True,
                # RTAB-Map 파라미터
                "Rtabmap/DetectionRate": "0.5",
                "Rtabmap/LoopClosureReextractFeatures": "true",
                "Reg/Strategy": "0",
                "RGBD/OptimizeMaxError": "3.0",
                "RGBD/ProximityPathMaxNeighbors": "10",
                "RGBD/AngularUpdate": "0.1",
                "RGBD/LinearUpdate": "0.1",
                "Reg/Force3DoF": "true",
                "Grid/FromDepth": "true",
                "Grid/RangeMax": "5.0",
                "Grid/CellSize": "0.05",
                "Grid/MaxGroundHeight": "0.05",
                "Grid/MaxObstacleHeight": "2.0",
                "Grid/NormalsSegmentation": "false",
                "Rtabmap/MemoryThr": "0",
                "Rtabmap/ImageBufferSize": "1",
            }
        ],
        remappings=camera_remappings + [
            ("odom", "/odom"),
            ("imu", "/imu/data"),
        ],
        arguments=["-d"],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation clock from /clock topic",
            ),
            base_to_camera_tf,
            camera_to_optical_tf,
            rtabmap_node,
        ]
    )
