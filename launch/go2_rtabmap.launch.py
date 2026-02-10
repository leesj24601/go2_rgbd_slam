"""
RTAB-Map RGBD SLAM launch file for Go2 + Isaac Sim.

Isaac Sim이 퍼블리시하는 카메라 토픽을 구독하여
Visual Odometry + SLAM + 3D/2D 맵 생성.

Usage:
    source /opt/ros/humble/setup.bash
    ros2 launch /home/cvr/Desktop/sj/isaac-project/launch/go2_rtabmap.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Isaac Sim 토픽 remapping (공통)
    camera_remappings = [
        ("rgb/image", "/camera/color/image_raw"),
        ("depth/image", "/camera/depth/image_rect_raw"),
        ("rgb/camera_info", "/camera/camera_info"),
    ]

    # --- static TF: odom → camera_link ---
    # 로봇의 초기 위치(odom)에서 카메라(go2 머리)까지의 변환
    # x=0.3m (전방), z=0.35m (지상고)
    # roll=-1.57 (카메라가 정면을 보도록 회전 보정)
    camera_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0.3",
            "0",
            "0.35",
            "-1.5708",
            "0",
            "-1.5708",
            "camera_link",
            "camera_optical_frame",
        ],
    )

    # --- rgbd_odometry 노드 ---
    # RGB+Depth로 Visual Odometry 계산
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
                "queue_size": 10,
                "use_sim_time": use_sim_time,
                "Odom/Strategy": "0",
                "Odom/GuessMotion": "true",
                "Odom/ResetCountdown": "5",
                "Odom/ImageDecimation": "2",
                "Vis/FeatureType": "8",
                "Vis/MaxFeatures": "800",
                "Vis/MinInliers": "10",
                "Vis/MaxDepth": "8.0",
                "GFTT/MinDistance": "3",
            }
        ],
        remappings=camera_remappings + [("odom", "/odom")],
    )

    # --- rtabmap SLAM 노드 ---
    # Visual Odometry + RGB-D → SLAM + 맵 생성
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
                "qos": 0,
                "queue_size": 10,
                "use_sim_time": use_sim_time,
                "Rtabmap/DetectionRate": "5",
                "Reg/Strategy": "0",
                "RGBD/OptimizeMaxError": "0",
                "Reg/Force3DoF": "true",  # 2D 평면 이동 가정 (Z축 고정)
                "Grid/FromDepth": "false",
                "Grid/RangeMax": "3.0",
                "Grid/CellSize": "0.1",
            }
        ],
        remappings=camera_remappings + [("odom", "/odom")],
        arguments=["-d"],  # 매 실행 시 DB 초기화 (개발 단계)
    )

    # --- rtabmap_viz 시각화 노드 (추가됨) ---
    # SLAM 과정, 루프 클로저, 특징점 매칭 등을 실시간으로 시각화
    # 주의: GPU 자원을 많이 사용하여 시뮬레이션 랙을 유발할 수 있음
    rtabmap_viz_node = Node(
        package="rtabmap_viz",
        executable="rtabmap_viz",
        output="screen",
        parameters=[
            {
                "frame_id": "camera_link",
                "odom_frame_id": "odom",
                "subscribe_depth": True,
                "subscribe_odom_info": True,
                "approx_sync": True,
                "queue_size": 10,
                "use_sim_time": use_sim_time,
            }
        ],
        remappings=camera_remappings + [("odom", "/odom")],
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
            # 시각화 노드 활성화 (랙이 걸리면 아래 라인을 주석 처리하세요)
            rtabmap_viz_node,
        ]
    )
