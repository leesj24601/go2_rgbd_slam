#!/usr/bin/env python3
"""
ROS2 Camera Bridge — Isaac Sim 카메라 데이터를 ROS2 토픽으로 퍼블리시.

시스템 Python 3.10 (/usr/bin/python3)으로 실행해야 합니다.
사용법:
    source /opt/ros/humble/setup.bash
    python3 scripts/ros2_camera_bridge.py
"""
import socket
import struct
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

IMG_H, IMG_W = 480, 640
BRIDGE_PORT = 5555


def recv_exactly(sock, nbytes):
    """소켓에서 정확히 nbytes만큼 수신."""
    buf = bytearray()
    while len(buf) < nbytes:
        chunk = sock.recv(nbytes - len(buf))
        if not chunk:
            raise ConnectionError("연결 종료")
        buf.extend(chunk)
    return bytes(buf)


def main():
    rclpy.init()
    node = rclpy.create_node("go2_camera")

    rgb_pub = node.create_publisher(Image, "/camera/color/image_raw", 10)
    depth_pub = node.create_publisher(Image, "/camera/depth/image_rect_raw", 10)
    info_pub = node.create_publisher(CameraInfo, "/camera/camera_info", 10)

    # CameraInfo (고정)
    cam_info = CameraInfo()
    cam_info.header.frame_id = "camera_link"
    cam_info.height = IMG_H
    cam_info.width = IMG_W
    fx = fy = 15.0 / 20.955 * IMG_W  # ~457.7
    cx, cy = IMG_W / 2.0, IMG_H / 2.0
    cam_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    cam_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    cam_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    cam_info.distortion_model = "plumb_bob"
    cam_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

    # TCP 서버
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", BRIDGE_PORT))
    server.listen(1)
    node.get_logger().info(f"Bridge 서버 대기 중 (port {BRIDGE_PORT})...")

    conn, addr = server.accept()
    node.get_logger().info(f"Isaac Sim 연결됨: {addr}")

    header_size = struct.calcsize("II")
    frame_count = 0

    try:
        while rclpy.ok():
            header = recv_exactly(conn, header_size)
            rgb_len, depth_len = struct.unpack("II", header)

            rgb_data = recv_exactly(conn, rgb_len)
            depth_data = recv_exactly(conn, depth_len)

            stamp = node.get_clock().now().to_msg()

            # RGB
            rgb_msg = Image()
            rgb_msg.header.stamp = stamp
            rgb_msg.header.frame_id = "camera_link"
            rgb_msg.height = IMG_H
            rgb_msg.width = IMG_W
            rgb_msg.encoding = "rgb8"
            rgb_msg.is_bigendian = False
            rgb_msg.step = IMG_W * 3
            rgb_msg.data = rgb_data
            rgb_pub.publish(rgb_msg)

            # Depth
            depth_msg = Image()
            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = "camera_link"
            depth_msg.height = IMG_H
            depth_msg.width = IMG_W
            depth_msg.encoding = "32FC1"
            depth_msg.is_bigendian = False
            depth_msg.step = IMG_W * 4
            depth_msg.data = depth_data
            depth_pub.publish(depth_msg)

            # CameraInfo
            cam_info.header.stamp = stamp
            info_pub.publish(cam_info)

            frame_count += 1
            if frame_count % 300 == 0:
                node.get_logger().info(f"퍼블리시 프레임: {frame_count}")

    except ConnectionError:
        node.get_logger().info("Isaac Sim 연결 종료")
    finally:
        conn.close()
        server.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
