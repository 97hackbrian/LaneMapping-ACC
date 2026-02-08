#!/usr/bin/env python3
"""
Depth Masker Node

Applies road segmentation mask to depth image, keeping only road regions.
Publishes masked depth for nvblox 3D reconstruction.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters
import cv2
import numpy as np


class DepthMaskerNode(Node):
    """
    Node that applies road segmentation mask to depth image.
    Only road regions will have valid depth values for nvblox.
    """

    def __init__(self):
        super().__init__('depth_masker')
        
        # Declare parameters
        self.declare_parameter('depth_input_topic', '/camera/depth_image')
        self.declare_parameter('mask_input_topic', '/segmentation/road_mask')
        self.declare_parameter('depth_output_topic', '/nvblox/depth/image')
        self.declare_parameter('sync_slop', 0.1)
        self.declare_parameter('resize_to_depth', True)
        self.declare_parameter('output_frame_id', 'camera_depth_optical_frame')  # Override frame_id
        self.declare_parameter('depth_scale', 1000.0)  # Divide by this to get meters (1000 for mm, 10000 for custom)
        
        # Get parameters
        depth_input = self.get_parameter('depth_input_topic').value
        mask_input = self.get_parameter('mask_input_topic').value
        self.depth_output = self.get_parameter('depth_output_topic').value
        sync_slop = self.get_parameter('sync_slop').value
        self.resize_to_depth = self.get_parameter('resize_to_depth').value
        self.output_frame_id = self.get_parameter('output_frame_id').value
        self.depth_scale = self.get_parameter('depth_scale').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # QoS profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Synchronized subscribers using message_filters
        self.depth_sub = message_filters.Subscriber(
            self, Image, depth_input, qos_profile=qos
        )
        self.mask_sub = message_filters.Subscriber(
            self, Image, mask_input, qos_profile=qos
        )
        
        # Approximate time synchronizer
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.depth_sub, self.mask_sub],
            queue_size=10,
            slop=sync_slop
        )
        self.ts.registerCallback(self.sync_callback)
        
        # Publisher
        self.depth_publisher = self.create_publisher(Image, self.depth_output, qos)
        
        self.get_logger().info(
            f'Depth Masker initialized. '
            f'Depth: {depth_input}, Mask: {mask_input} -> {self.depth_output}'
        )

    def sync_callback(self, depth_msg: Image, mask_msg: Image):
        """Process synchronized depth and mask images."""
        try:
            # Convert depth image
            # mono16/16UC1 are in millimeters, nvblox expects meters for 32FC1
            if depth_msg.encoding in ['32FC1']:
                # Already in meters as float
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
                depth_in_meters = depth_image
                self.get_logger().info(f'Depth already 32FC1, range: {np.min(depth_image):.3f} - {np.max(depth_image):.3f} m')
            elif depth_msg.encoding in ['16UC1', 'mono16']:
                # 16-bit depth, convert using configurable scale
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
                # Convert to meters using depth_scale parameter
                depth_in_meters = depth_image.astype(np.float32) / self.depth_scale
                # Log conversion for debugging
                raw_max = np.max(depth_image)
                converted_max = np.max(depth_in_meters)
                self.get_logger().info(f'Depth converted: raw_max={raw_max} / {self.depth_scale} -> {converted_max:.3f} m')
            else:
                # Try passthrough for other encodings
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
                depth_in_meters = depth_image.astype(np.float32)
                self.get_logger().warn(f'Unknown depth encoding: {depth_msg.encoding}')
            
            # Convert mask image
            mask_image = self.bridge.imgmsg_to_cv2(mask_msg, desired_encoding='mono8')
            
            # Resize mask to match depth dimensions if needed
            depth_h, depth_w = depth_in_meters.shape[:2]
            mask_h, mask_w = mask_image.shape[:2]
            
            if (depth_h != mask_h or depth_w != mask_w) and self.resize_to_depth:
                mask_image = cv2.resize(
                    mask_image, 
                    (depth_w, depth_h), 
                    interpolation=cv2.INTER_NEAREST
                )
                self.get_logger().debug(
                    f'Resized mask from ({mask_w}x{mask_h}) to ({depth_w}x{depth_h})'
                )
            
            # Create binary mask (threshold at 128 to handle any edge cases)
            binary_mask = (mask_image > 128).astype(np.uint8)
            
            # Apply mask to depth image
            # Where mask is 0 (not road), set depth to NaN (nvblox ignores NaN values)
            masked_depth = np.where(binary_mask == 1, depth_in_meters, np.nan).astype(np.float32)
            
            # Convert back to ROS Image message as 32FC1 (meters) for nvblox
            masked_msg = self.bridge.cv2_to_imgmsg(masked_depth, encoding='32FC1')
            masked_msg.header = depth_msg.header  # Preserve timestamp
            masked_msg.header.frame_id = self.output_frame_id  # Override frame_id to match camera_info
            
            # Publish masked depth
            self.depth_publisher.publish(masked_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing depth/mask: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DepthMaskerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
