from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'yolo_realsense'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
        (os.path.join('share', package_name, 'models'), glob('models/*.onnx')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'models/openvino'), glob('models/yolov11s_openvino_model/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='linboxi123',
    maintainer_email='linboxi123@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'yolo_realsense_node = yolo_realsense.detection_node:main',
        ],
    },
)
