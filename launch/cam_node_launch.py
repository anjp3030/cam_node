#!/usr/bin/env python3
import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 첫 번째 카메라 노드 (예: UV570)
        Node(
            package='cam_node',
            executable='cam_node',
            name='cam_node_camera1',
            output='screen',
            parameters=[{
                'device_path': '/dev/uvc-hd',
                'camera_name': 'uv570'
            }]
        ),
        # 두 번째 카메라 노드 (예: Femto, udev 규칙에 따른 장치 경로 사용)
        Node(
            package='cam_node',
            executable='cam_node',
            name='cam_node_camera2',
            output='screen',
            parameters=[{
                'device_path': '/dev/uvc-bolt',
                'camera_name': 'bolt'
            }]
        ),
        # 세 번째 카메라 노드 (예: Volt 등 다른 카메라)
        Node(
            package='cam_node',
            executable='cam_node',
            name='cam_node_camera3',
            output='screen',
            parameters=[{
                'device_path': '/dev/uvc-d435',
                'camera_name': 'd435'
            }]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
