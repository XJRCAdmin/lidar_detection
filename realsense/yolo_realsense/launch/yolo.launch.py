import torch
from ament_index_python import get_package_share_directory
import os
from launch import LaunchDescription
from launch_ros.actions import Node


torch_device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

def generate_launch_description():
    this_package_name='yolo_realsense'
    model_path = os.path.join(
        get_package_share_directory(this_package_name),
        'models',
        'yolov11s.pt'
    ) 
    # Run the yolov11 node, with the set device
    yolov11_node = Node(
        package=this_package_name,
        executable='yolo_realsense_node',
        #name='node2', # Default is name of executable
        output='screen',
        parameters=[
            {'device': f'{torch_device}'},
            {'model': model_path},  
            {'enable_bg_removal': True},
            {'model_type': 'pt'}
        ],
    )
    
    # Launch them all!
    return LaunchDescription([
        yolov11_node,
    ])