from setuptools import setup
import os
from glob import glob

package_name = 'cam_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # ament index resource
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # package.xml 설치
        ('share/' + package_name, ['package.xml']),
        # launch 파일들 (있는 경우)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='siheon',
    maintainer_email='anjp3030@naver.com',
    description='A ROS2 package that publishes camera images using OpenCV and cv_bridge',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cam_node = cam_node.cam_node:main'
        ],
    },
)
