#!/usr/bin/env python3
"""
Road Segmentation Launch File

Launches the 3 nodes for road segmentation and depth masking:
1. road_mask_extractor_node - Extract blue road from color mask
2. depth_masker_node - Apply mask to depth image
3. camera_info_publisher_node - Generate CameraInfo for nvblox
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package directory
    pkg_dir = get_package_share_directory('lane_mapping_acc')
    
    # Config file path
    config_file = os.path.join(pkg_dir, 'config', 'segmentation_params.yaml')
    
    
    # Launch arguments
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to segmentation parameters YAML file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    
    # Road Mask Extractor Node
    road_mask_extractor = Node(
        package='lane_mapping_acc',
        executable='road_mask_extractor_node.py',
        name='road_mask_extractor',
        output='screen',
        parameters=[LaunchConfiguration('config_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[]
    )
    
    # Depth Masker Node
    depth_masker = Node(
        package='lane_mapping_acc',
        executable='depth_masker_node.py',
        name='depth_masker',
        output='screen',
        parameters=[LaunchConfiguration('config_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[]
    )
    
    # Camera Info Publisher Node
    camera_info_publisher = Node(
        package='lane_mapping_acc',
        executable='camera_info_publisher_node.py',
        name='camera_info_publisher',
        output='screen',
        parameters=[LaunchConfiguration('config_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[]
    )
    
    return LaunchDescription([
        config_arg,
        use_sim_time_arg,
        road_mask_extractor,
        depth_masker,
        camera_info_publisher,
    ])
