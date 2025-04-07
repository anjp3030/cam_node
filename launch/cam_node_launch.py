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
                'device_path': '/dev/v4l/by-id/usb-HD_Camera_HD_Camera-video-index0',
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
                'device_path': '/dev/v4l/by-id/usb-Orbbec_Orbbec_Femto_Bolt_3D_Camera_CL8384200Y6-video-index0',
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
                'device_path': '/dev/v4l/by-path/pci-0000:00:14.0-usb-0:2:1.3-video-index0',  # 실제 사용 환경에 맞게 수정하세요.
                'camera_name': 'd435'
            }]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
