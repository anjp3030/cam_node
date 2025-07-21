# cam_node

**cam_node**는 ROS2 환경에서 여러 종류의 카메라(Femto bolt(exo), realsense d435i(hand), uv570(lateral))의 이미지를 캡처하여 ROS2 토픽으로 publish하는 패키지입니다. OpenCV와 cv_bridge를 사용하여 카메라에서 영상을 받아 ROS2 메시지로 변환합니다.

---

## 주요 기능

- 여러 카메라(uv570, bolt, d435 등) 지원
- 각 카메라별로 독립적인 노드 실행 및 토픽 publish(`/camera_name/image_raw`)
- 카메라 장치 경로와 이름은 파라미터로 지정 가능
- OpenCV를 사용한 영상 캡처 및 변환
- ROS2 Image 토픽으로 실시간 publish

---

## 설치 및 의존성

```bash
# ROS2 패키지 빌드 환경에서
git clone https://github.com/anjp3030/cam_node.git
cd cam_node
rosdep install --from-paths . --ignore-src -r -y
colcon build
```

**의존 패키지:**
- Python 3
- OpenCV (`cv2`)
- ROS2 (`rclpy`, `sensor_msgs`)
- cv_bridge

---

## 사용법

### 1. 단일 카메라 노드 실행 예시

```bash
ros2 run cam_node cam_node --ros-args -p device_path:=/dev/uvc-hd -p camera_name:=uv570
```

### 2. launch 파일로 여러 카메라 노드 실행

`launch/cam_node_launch.py` 파일을 이용해 여러 카메라를 동시에 실행할 수 있습니다.

```bash
ros2 launch cam_node cam_node_launch.py
```

#### path 예시 
'''
 # 세 번째 카메라 노드 (예: Volt 등 다른 카메라)
        Node(
            package='cam_node',
            executable='cam_node',
            name='cam_node_camera3',
            output='screen',
            parameters=[{
                'device_path': '/dev/uvc-d435', #device path udev rule 로 uvc-d435로 설정
                'camera_name': 'd435'
            }]
        )
'''

- 각 카메라별 장치 경로와 이름은 launch 파일에서 지정합니다.

### 3. 토픽 예시

- `/uv570/image_raw`
- `/bolt/image_raw`
- `/d435/image_raw`

---

## 예시 코드

카메라 노드는 다음과 같이 동작합니다:
```python
class CameraPublisher(Node):
    def __init__(self):
        ...
        self.declare_parameter("device_path", "/dev/v4l/by-id/usb-HD_Camera_HD_Camera-video-index0")
        self.declare_parameter("camera_name", "uv570")
        ...
        self.publisher_ = self.create_publisher(Image, publish_topic, 10)
        self.cap = cv2.VideoCapture(device_path)
        ...
    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(image_msg)
```

---

## 라이선스

이 프로젝트는 Apache License 2.0을 따릅니다.

---

## Maintainer

- siheon ([anjp3030@naver.com](mailto:anjp3030@naver.com))
