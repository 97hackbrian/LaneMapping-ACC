#!/usr/bin/env python3
"""
Nvblox Road Mapping Launch File

Launches the complete pipeline for 3D road mapping:
1. Static Transform Publisher (Camera to Base)
2. Color Segmentation Node (QCar2) - with roi_height_ratio=0.2
3. Road Segmentation Launch (Mask Extractor, Depth Masker, Camera Info)
4. Cartographer Mapping
5. Nvblox Node
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
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # 1. Static Transform Publisher
    # ros2 run tf2_ros static_transform_publisher --x 0.095 --y 0.032 --z 0.172 --roll -1.5708 --pitch 0 --yaw -1.5708 --frame-id base_link --child-frame-id camera_depth_optical_frame
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        output='screen',
        arguments=[
            '--x', '0.095', '--y', '0.032', '--z', '0.172',
            '--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_depth_optical_frame'
        ]
    )

    # 2. Color Segmentation Node
    # ros2 run qcar2_laneseg_acc color_segmentation_node.py --ros-args -p roi_height_ratio:=0.2
    # Note: Using parameters directly. 'use_sim_time' is included for consistency with other nodes.
    # If the user specifically removed it, it might be due to a specific driver behavior, but 
    # for general ROS2 consistency it's usually safer to include it if simulation is possible.
    # However, since the user removed it in their edit, I will omit it here to respect their change
    # and to potentially avoid the issue they were facing.
    color_segmentation_node = Node(
        package='qcar2_laneseg_acc',
        executable='color_segmentation_node.py',
        name='color_segmentation',
        output='screen',
        parameters=[
            LaunchConfiguration('segmentation_config'),
            {'roi_height_ratio': 0.2},
            # {'use_sim_time': LaunchConfiguration('use_sim_time')} # Omitting as per user's last edit pattern
        ]
    )

    # 3. Include Road Segmentation Launch
    # Includes: road_mask_extractor, depth_masker, camera_info_publisher
    # This was reported to "work well" by the user.
    road_segmentation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'road_segmentation.launch.py')
        ),
        launch_arguments={
            'config_file': LaunchConfiguration('segmentation_config'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # 4. Cartographer Mapping
    # ros2 launch lane_mapping_acc cartographer_mapping.launch.py
    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'cartographer_mapping.launch.py')
        ),
        launch_arguments={
            'use_sim': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # 5. Nvblox Node
    # ros2 launch lane_mapping_acc nvblox_road_mapping.launch.py use_sim_time:=true
    nvblox_node = Node(
        package='nvblox_ros',
        executable='nvblox_node',
        name='nvblox_node',
        output='screen',
        parameters=[
            LaunchConfiguration('nvblox_config'),
            {
                'global_frame': LaunchConfiguration('global_frame'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ],
        remappings=[
            # Remap depth input to our masked depth
            ('depth/image', '/nvblox/depth/image'),
            ('depth/camera_info', '/nvblox/depth/camera_info'),
            # Use filtered odometry for mapping
            ('pose', '/odom_filtered_pose'),
        ]
    )
    
    return LaunchDescription([
        segmentation_config_arg,
        nvblox_config_arg,
        global_frame_arg,
        use_sim_time_arg,
        static_tf_publisher,
        color_segmentation_node,
        road_segmentation_launch,
        cartographer_launch,
        nvblox_node,
    ])
