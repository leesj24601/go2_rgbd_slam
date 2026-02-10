"""
RTAB-Map RGBD SLAM launch file for Go2 + Isaac Sim (Lightweight version).

Isaac Sim이 퍼블리시하는 칩메라 토픽을 구독하여 Visual Odometry + SLAM.
rtabmap_viz 제거 버전 (성능 최적화)

Usage:
    source /opt/ros/humble/setup.bash
    ros2 launch /home/cvr/Desktop/sj/isaac-project/launch/go2_rtabmap_minimal.launch.py
"""

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

    camera_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0",
            "0",
            "0",
            "-1.5707963",
            "0",
            "-1.5707963",
            "camera_link",
            "camera_optical_frame",
        ],
    )

    rgbd_odometry_node = Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        output="screen",
        parameters=[
            {
                "frame_id": "camera_link",
                "odom_frame_id": "odom",
                "publish_tf": True,
                "wait_for_transform": 1.0,
                "approx_sync": True,
                "approx_sync_max_interval": 0.1,
                "subscribe_rgbd": False,
                "qos": 0,
                "use_sim_time": use_sim_time,
                "Odom/Strategy": "0",
                "Odom/GuessMotion": "true",
                "Odom/ResetCountdown": "5",
                "Vis/FeatureType": "8",
                "Vis/MaxFeatures": "1000",
                "Vis/MinInliers": "10",
                "Vis/MaxDepth": "8.0",
                "GFTT/MinDistance": "3",
            }
        ],
        remappings=camera_remappings + [("odom", "/odom")],
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
                "subscribe_odom_info": True,
                "approx_sync": True,
                "approx_sync_max_interval": 0.1,
                "publish_tf": True,
                "qos": 0,  # Reliable (Isaac Sim 브릿지 기본값과 일치)
                "use_sim_time": use_sim_time,
                "Reg/Strategy": "0",
                "RGBD/OptimizeMaxError": "0",
                "Reg/Force3DoF": "false",
                "Grid/FromDepth": "false",
                "Grid/RangeMax": "3.0",
                "Grid/CellSize": "0.1",
            }
        ],
        remappings=camera_remappings + [("odom", "/odom")],
        arguments=["-d"],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation clock from /clock topic",
            ),
            camera_tf_node,
            rgbd_odometry_node,
            rtabmap_node,
        ]
    )
