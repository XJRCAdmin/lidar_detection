
import os
import torch
torch_device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    this_package_name='yolo_realsense'
    realsense_package_name = 'realsense2_camera'
    
    model_path = os.path.join(
        get_package_share_directory(this_package_name),
        'models',
        'yolov11s.pt'
    )
    rviz_path = os.path.join(
        get_package_share_directory(this_package_name),
        'rviz',
        'yolo.rviz'
    )

    # Launch Realsense camera launch file with aligned depth images publisher
    rs_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(realsense_package_name), 'launch', 'rs_launch.py'
        )]), launch_arguments={'align_depth.enable': 'true','bg_removal.enable': 'true'}.items()
    )
    
    
    # Run the yolov11 node, with the set device
    yolov11_node = Node(
        package=this_package_name,
        executable='yolo_realsense_node',
        #name='node2',
        output='screen',
        parameters=[
            {'device': f'{torch_device}'},
            {'model': model_path},  
        ],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_path],
        output='screen'
    )

    return LaunchDescription([
        rs_camera,
        yolov11_node,
        rviz_node
    ])