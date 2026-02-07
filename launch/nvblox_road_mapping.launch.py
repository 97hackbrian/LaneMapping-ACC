#!/usr/bin/env python3
"""
Nvblox Road Mapping Launch File

Launches the complete pipeline for 3D road mapping:
1. Road segmentation nodes (via include)
2. Nvblox node configured for static reconstruction (no color)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    pkg_dir = get_package_share_directory('lane_mapping_acc')
    
    # Config files
    segmentation_config = os.path.join(pkg_dir, 'config', 'segmentation_params.yaml')
    nvblox_config = os.path.join(pkg_dir, 'config', 'nvblox_static.yaml')
    
    # Launch arguments
    segmentation_config_arg = DeclareLaunchArgument(
        'segmentation_config',
        default_value=segmentation_config,
        description='Path to segmentation parameters YAML file'
    )
    
    nvblox_config_arg = DeclareLaunchArgument(
        'nvblox_config',
        default_value=nvblox_config,
        description='Path to nvblox configuration YAML file'
    )
    
    global_frame_arg = DeclareLaunchArgument(
        'global_frame',
        default_value='map',
        description='Global frame for nvblox mapping'
    )
    
    # Include road segmentation launch
    road_segmentation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'road_segmentation.launch.py')
        ),
        launch_arguments={
            'config_file': LaunchConfiguration('segmentation_config')
        }.items()
    )
    
    # Nvblox Node with remappings for our masked depth
    nvblox_node = Node(
        package='nvblox_ros',
        executable='nvblox_node',
        name='nvblox_node',
        output='screen',
        parameters=[
            LaunchConfiguration('nvblox_config'),
            {'global_frame': LaunchConfiguration('global_frame')}
        ],
        remappings=[
            # Remap depth input to our masked depth
            ('depth/image', '/nvblox/depth/image'),
            ('depth/camera_info', '/nvblox/depth/camera_info'),
            # We don't use color for static reconstruction
            # ('color/image', '/camera/color_image'),
            # ('color/camera_info', '/camera/color/camera_info'),
        ]
    )
    
    return LaunchDescription([
        segmentation_config_arg,
        nvblox_config_arg,
        global_frame_arg,
        road_segmentation_launch,
        nvblox_node,
    ])
