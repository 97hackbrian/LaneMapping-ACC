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
        
        # Get parameters
        depth_input = self.get_parameter('depth_input_topic').value
        mask_input = self.get_parameter('mask_input_topic').value
        self.depth_output = self.get_parameter('depth_output_topic').value
        sync_slop = self.get_parameter('sync_slop').value
        self.resize_to_depth = self.get_parameter('resize_to_depth').value
        
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
            # Convert depth image (preserve original encoding)
            if depth_msg.encoding == '32FC1':
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
            elif depth_msg.encoding == '16UC1':
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
            else:
                # Try passthrough
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            
            # Convert mask image
            mask_image = self.bridge.imgmsg_to_cv2(mask_msg, desired_encoding='mono8')
            
            # Resize mask to match depth dimensions if needed
            depth_h, depth_w = depth_image.shape[:2]
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
            # Where mask is 0 (not road), set depth to 0 (invalid)
            if depth_image.dtype == np.float32:
                masked_depth = np.where(binary_mask == 1, depth_image, 0.0).astype(np.float32)
            else:
                masked_depth = np.where(binary_mask == 1, depth_image, 0).astype(depth_image.dtype)
            
            # Convert back to ROS Image message
            masked_msg = self.bridge.cv2_to_imgmsg(masked_depth, encoding=depth_msg.encoding)
            masked_msg.header = depth_msg.header  # Preserve timestamp and frame_id
            
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
