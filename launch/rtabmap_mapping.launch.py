import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Path to config for camera info parameters
    pkg_dir = get_package_share_directory('lane_mapping_acc')
    config_file = os.path.join(pkg_dir, 'config', 'segmentation_params.yaml')
    
    # RGB Camera Info Publisher (Synthetic) - For Odometry (High Rate)
    rgb_camera_info_publisher = Node(
        package='lane_mapping_acc',
        executable='camera_info_publisher_node.py',
        name='rgb_camera_info_publisher',
        output='screen',
        parameters=[
            config_file, 
            {
                'use_sim_time': use_sim_time,
                'image_topic': '/camera/color_image',
                'camera_info_topic': '/camera/color/camera_info',
                'frame_id': 'camera_depth_optical_frame'
            }
        ]
    )

    # RGB Camera Info Publisher (Synthetic) - For SLAM (Throttled Rate)
    rgb_throttled_camera_info_publisher = Node(
        package='lane_mapping_acc',
        executable='camera_info_publisher_node.py',
        name='rgb_throttled_camera_info_publisher',
        output='screen',
        parameters=[
            config_file, 
            {
                'use_sim_time': use_sim_time,
                'image_topic': '/camera/color_image_throttled',
                'camera_info_topic': '/camera/color/camera_info_throttled', # New topic for SLAM
                'frame_id': 'camera_depth_optical_frame'
            }
        ]
    )

    # Static Transform Publisher (as requested by user)
    # base_link -> camera_depth_optical_frame
    # RGB Throttler
    # Reduces RGB rate to match masked depth (~6Hz)
    rgb_throttler_node = Node(
        package='lane_mapping_acc',
        executable='rgb_throttler_node.py',
        name='rgb_throttler',
        output='screen',
        parameters=[{
            'input_topic': '/camera/color_image',
            'output_topic': '/camera/color_image_throttled',
            'target_fps': 6.0
        }]
    )

    # Static Transform: camera_depth_optical_frame -> color_image
    # Required because RGB image frame is 'color_image' but no TF exists for it
    static_tf_color_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='color_frame_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'camera_depth_optical_frame',
            '--child-frame-id', 'color_image'
        ]
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_depth_tf',
        arguments=[
            '--x', '0.095',
            '--y', '0.032',
            '--z', '0.172',
            '--roll', '-1.5708',
            '--pitch', '0',
            '--yaw', '-1.5708',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_depth_optical_frame'
        ]
    )

    # Visual Odometry Node
    # Uses raw depth and RGB to compute odometry
    rtabmap_odom_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'subscribe_depth': True,
            'use_sim_time': use_sim_time,
            'wait_imu_to_init': False,
            'qos_image': 2,
            'qos_camera_info': 2,
            'qos_imu': 2,
            'approx_sync': True,
            'approx_sync_max_interval': 0.1, # Allow up to 100ms jitter
            'queue_size': 50,
            'topic_queue_size': 50,
            'sync_queue_size': 50,
            'wait_for_transform': 0.2
        }],
        remappings=[
            ('rgb/image', '/camera/color_image'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/depth_image'), # RAW depth for odometry
            ('odom', '/odom')
        ]
    )

    # RTAB-Map SLAM Node
    # Uses MASKED depth for mapping to ignore non-road areas
    rtabmap_slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'map_frame_id': 'map',
            'subscribe_depth': True,
            'subscribe_scan': False,
            'use_sim_time': use_sim_time,
            'qos_image': 2,
            'qos_camera_info': 2,
            'qos_imu': 2,
            'approx_sync': True,
            'approx_sync_max_interval': 0.1, # Allow up to 100ms jitter
            'queue_size': 50,
            'topic_queue_size': 50,
            'sync_queue_size': 50,
            'wait_for_transform': 0.2,
            # Robustness parameters
            'Vis/MinInliers': '12',
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'true'
        }],
        remappings=[
            ('rgb/image', '/camera/color_image_throttled'),
            ('rgb/camera_info', '/camera/color/camera_info_throttled'), # Throttled Info
            ('depth/image', '/nvblox/depth/image'), # MASKED depth for mapping
            ('odom', '/odom'),
            ('imu', '/qcar2_imu')
        ],
        arguments=['-d'] # Delete database on start
    )

    # RTAB-Map Visualization (Optional, can be removed if specific to headless)
    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        output='screen',
        parameters=[{
            'subscribe_depth': True,
            'subscribe_odom_info': True,
            'frame_id': 'base_link',
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/nvblox/depth/image'), # Visualize the masked depth
            ('odom', '/odom')
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        rgb_throttler_node,
        rgb_camera_info_publisher,
        rgb_throttled_camera_info_publisher,
        static_tf_node,
        static_tf_color_node,
        rtabmap_odom_node,
        rtabmap_slam_node,
        # rtabmap_viz_node # Uncomment to enable visualization by default
    ])
