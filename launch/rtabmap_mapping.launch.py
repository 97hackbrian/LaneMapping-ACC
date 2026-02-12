import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Path to config for camera info parameters
    pkg_dir = get_package_share_directory('lane_mapping_acc')
    config_file = os.path.join(pkg_dir, 'config', 'segmentation_params.yaml')

    # ══════════════════════════════════════════════════════════════════
    # 1. CAMERA INFO PUBLISHER (Synthetic)
    # Generates CameraInfo from RGB image headers (no real CameraInfo exists)
    # ══════════════════════════════════════════════════════════════════
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

    # ══════════════════════════════════════════════════════════════════
    # 2. DEPTH ENCODING FIXER
    # Converts mono16 -> 32FC1 (float meters) + NaN for zeros
    # Without this, RTAB-Map crashes with SIGFPE
    # ══════════════════════════════════════════════════════════════════
    depth_encoding_fixer_node = Node(
        package='lane_mapping_acc',
        executable='depth_encoding_fixer_node.py',
        name='depth_encoding_fixer',
        output='screen',
        parameters=[{
            'input_topic': '/camera/depth_image',
            'output_topic': '/camera/depth_image_fixed',
            'depth_scale': 0.001  # mono16 millimeters -> meters
        }]
    )

    # ══════════════════════════════════════════════════════════════════
    # 3. STATIC TRANSFORMS
    # ══════════════════════════════════════════════════════════════════
    # base_link -> camera_depth_optical_frame (camera mount)
    static_tf_camera = Node(
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

    # camera_depth_optical_frame -> color_image (identity, same physical camera)
    static_tf_color = Node(
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

    # camera_depth_optical_frame -> depth_image (identity, same physical camera)
    static_tf_depth = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='depth_frame_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'camera_depth_optical_frame',
            '--child-frame-id', 'depth_image'
        ]
    )

    # ══════════════════════════════════════════════════════════════════
    # 4. WHEEL ODOMETRY (IMU + Encoders)
    # Replaces visual odometry - stable, no crashes
    # ══════════════════════════════════════════════════════════════════
    wheel_odom_node = Node(
        package='lane_mapping_acc',
        executable='joint_to_odom_node.py',
        name='joint_to_odom',
        output='screen',
        parameters=[{
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'publish_tf': True,
            'calibration_samples': 200
        }]
    )

    # ══════════════════════════════════════════════════════════════════
    # 5. RTAB-Map SLAM
    # Uses wheel odometry + RGB + fixed depth for 3D mapping
    # No throttling needed - RTAB-Map limits its own processing rate
    # ══════════════════════════════════════════════════════════════════
    rtabmap_slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'subscribe_depth': True,
            'subscribe_scan': False,
            'use_sim_time': use_sim_time,
            # QoS - all Best Effort to match sensor sources
            'qos_image': 2,       # Best Effort
            'qos_camera_info': 2, # Best Effort
            'qos_odom': 2,        # Best Effort (match high-rate odom)
            # Sync settings
            'approx_sync': True,
            'approx_sync_max_interval': 0.5,  # Very permissive for rate differences
            'queue_size': 100,
            'topic_queue_size': 100,
            'sync_queue_size': 100,
            'wait_for_transform': 0.5,
            # Rate limiting (RTAB-Map processes at most 1 Hz)
            'Rtabmap/DetectionRate': '1.0',
            # Robustness parameters
            'Reg/Force3DoF': 'true',         # 2D SLAM for ground vehicle
            'Vis/MinInliers': '10',
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'true',
            'Vis/MinDepth': '0.3',
            'Vis/MaxDepth': '8.0'
        }],
        remappings=[
            ('rgb/image', '/camera/color_image'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/depth_image_fixed'),
            ('odom', '/odom'),
            ('imu', '/qcar2_imu')
        ],
        arguments=['-d']  # Delete database on start
    )

    # ══════════════════════════════════════════════════════════════════
    # 6. RTAB-Map Visualization (Optional)
    # ══════════════════════════════════════════════════════════════════
    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        output='screen',
        parameters=[{
            'subscribe_depth': True,
            'subscribe_odom_info': True,
            'frame_id': 'base_link',
            'use_sim_time': use_sim_time,
            'qos_image': 2,
            'qos_camera_info': 2,
            'qos_odom': 2
        }],
        remappings=[
            ('rgb/image', '/camera/color_image'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/depth_image_fixed'),
            ('odom', '/odom')
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        # Core pipeline
        rgb_camera_info_publisher,
        depth_encoding_fixer_node,
        static_tf_camera,
        static_tf_color,
        static_tf_depth,
        wheel_odom_node,
        rtabmap_slam_node,
        # rtabmap_viz_node  # Uncomment for visualization
    ])
