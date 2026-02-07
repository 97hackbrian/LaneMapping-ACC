#!/usr/bin/env python3
"""
Road Mask Extractor Node

Extracts blue road segmentation from /segmentation/color_mask
and publishes a binary mask for depth masking.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class RoadMaskExtractorNode(Node):
    """
    Node that extracts blue-colored road regions from a segmentation mask
    and publishes a binary mask.
    """

    def __init__(self):
        super().__init__('road_mask_extractor')
        
        # Declare parameters with defaults
        self.declare_parameter('blue_threshold_r', 0)
        self.declare_parameter('blue_threshold_g', 0)
        self.declare_parameter('blue_threshold_b', 255)
        self.declare_parameter('color_tolerance', 30)
        self.declare_parameter('input_topic', '/segmentation/color_mask')
        self.declare_parameter('output_mask_topic', '/segmentation/road_mask')
        self.declare_parameter('output_image_topic', '/segmentation/color_mask/road/image')
        
        # Get parameters
        self.target_r = self.get_parameter('blue_threshold_r').value
        self.target_g = self.get_parameter('blue_threshold_g').value
        self.target_b = self.get_parameter('blue_threshold_b').value
        self.tolerance = self.get_parameter('color_tolerance').value
        
        input_topic = self.get_parameter('input_topic').value
        output_mask_topic = self.get_parameter('output_mask_topic').value
        output_image_topic = self.get_parameter('output_image_topic').value
        
        self.get_logger().info(
            f'Target color (RGB): ({self.target_r}, {self.target_g}, {self.target_b}), '
            f'Tolerance: {self.tolerance}'
        )
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # QoS profile for sensor data
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            qos
        )
        
        # Publishers
        self.mask_publisher = self.create_publisher(Image, output_mask_topic, qos)
        self.image_publisher = self.create_publisher(Image, output_image_topic, qos)
        
        self.get_logger().info(
            f'Road Mask Extractor initialized. '
            f'Subscribing to: {input_topic}, '
            f'Publishing mask to: {output_mask_topic}, '
            f'Publishing image to: {output_image_topic}'
        )

    def image_callback(self, msg: Image):
        """Process incoming segmentation image and extract blue road regions."""
        try:
            # Convert ROS Image to OpenCV (assuming RGB8 or BGR8)
            if msg.encoding == 'rgb8':
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
                # Image is already in RGB
            elif msg.encoding == 'bgr8':
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                # Convert BGR to RGB for processing
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            else:
                # Try to convert to RGB
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            
            # Create mask for blue color detection
            # Calculate bounds with tolerance
            lower = np.array([
                max(0, self.target_r - self.tolerance),
                max(0, self.target_g - self.tolerance),
                max(0, self.target_b - self.tolerance)
            ], dtype=np.uint8)
            
            upper = np.array([
                min(255, self.target_r + self.tolerance),
                min(255, self.target_g + self.tolerance),
                min(255, self.target_b + self.tolerance)
            ], dtype=np.uint8)
            
            # Create binary mask (255 for road, 0 for non-road)
            mask = cv2.inRange(cv_image, lower, upper)
            
            # Optional: Apply morphological operations to clean up mask
            kernel = np.ones((3, 3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            # Publish binary mask
            mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
            mask_msg.header = msg.header
            self.mask_publisher.publish(mask_msg)
            
            # Create visualization image (road regions highlighted)
            road_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)
            
            # Publish road-only color image
            road_msg = self.bridge.cv2_to_imgmsg(road_image, encoding='rgb8')
            road_msg.header = msg.header
            self.image_publisher.publish(road_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = RoadMaskExtractorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
