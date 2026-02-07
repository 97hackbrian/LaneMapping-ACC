#!/usr/bin/env python3
"""
Camera Info Publisher Node

Generates and publishes synthetic CameraInfo messages for the masked depth image.
Required by nvblox for 3D reconstruction.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np


class CameraInfoPublisherNode(Node):
    """
    Node that generates CameraInfo messages synchronized with depth images.
    Uses configurable camera intrinsic parameters.
    """

    def __init__(self):
        super().__init__('camera_info_publisher')
        
        # Declare parameters
        self.declare_parameter('image_topic', '/nvblox/depth/image')
        self.declare_parameter('camera_info_topic', '/nvblox/depth/camera_info')
        self.declare_parameter('frame_id', 'camera_depth_optical_frame')
        
        # Camera intrinsics (RealSense D435 typical defaults)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fx', 386.23)
        self.declare_parameter('fy', 386.23)
        self.declare_parameter('cx', 323.48)
        self.declare_parameter('cy', 238.67)
        
        # Distortion coefficients
        self.declare_parameter('k1', 0.0)
        self.declare_parameter('k2', 0.0)
        self.declare_parameter('p1', 0.0)
        self.declare_parameter('p2', 0.0)
        self.declare_parameter('k3', 0.0)
        
        # Get parameters
        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value
        
        self.k1 = self.get_parameter('k1').value
        self.k2 = self.get_parameter('k2').value
        self.p1 = self.get_parameter('p1').value
        self.p2 = self.get_parameter('p2').value
        self.k3 = self.get_parameter('k3').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # QoS profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriber to depth image (to get timestamp and actual dimensions)
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos
        )
        
        # Publisher
        self.camera_info_publisher = self.create_publisher(
            CameraInfo, 
            camera_info_topic, 
            qos
        )
        
        self.get_logger().info(
            f'Camera Info Publisher initialized. '
            f'Subscribing to: {image_topic}, '
            f'Publishing to: {camera_info_topic}, '
            f'Frame ID: {self.frame_id}'
        )
        self.get_logger().info(
            f'Camera intrinsics: {self.width}x{self.height}, '
            f'fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}'
        )

    def image_callback(self, msg: Image):
        """Generate and publish CameraInfo synchronized with image."""
        try:
            # Create CameraInfo message
            camera_info = CameraInfo()
            
            # Header - use same timestamp as image for synchronization
            camera_info.header = msg.header
            camera_info.header.frame_id = self.frame_id
            
            # Use actual image dimensions if available, otherwise use parameters
            actual_width = msg.width if msg.width > 0 else self.width
            actual_height = msg.height if msg.height > 0 else self.height
            
            # Update intrinsics if resolution differs from configured
            fx, fy, cx, cy = self.fx, self.fy, self.cx, self.cy
            if actual_width != self.width or actual_height != self.height:
                # Scale intrinsics proportionally
                scale_x = actual_width / self.width
                scale_y = actual_height / self.height
                fx = self.fx * scale_x
                fy = self.fy * scale_y
                cx = self.cx * scale_x
                cy = self.cy * scale_y
            
            camera_info.width = actual_width
            camera_info.height = actual_height
            
            # Distortion model
            camera_info.distortion_model = 'plumb_bob'
            camera_info.d = [self.k1, self.k2, self.p1, self.p2, self.k3]
            
            # Intrinsic camera matrix (K) - 3x3 row-major
            camera_info.k = [
                fx,  0.0, cx,
                0.0, fy,  cy,
                0.0, 0.0, 1.0
            ]
            
            # Rectification matrix (R) - 3x3 identity for monocular
            camera_info.r = [
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0
            ]
            
            # Projection matrix (P) - 3x4
            camera_info.p = [
                fx,  0.0, cx,  0.0,
                0.0, fy,  cy,  0.0,
                0.0, 0.0, 1.0, 0.0
            ]
            
            # Binning (no binning)
            camera_info.binning_x = 0
            camera_info.binning_y = 0
            
            # ROI (full image)
            camera_info.roi.x_offset = 0
            camera_info.roi.y_offset = 0
            camera_info.roi.height = 0
            camera_info.roi.width = 0
            camera_info.roi.do_rectify = False
            
            # Publish
            self.camera_info_publisher.publish(camera_info)
            
        except Exception as e:
            self.get_logger().error(f'Error generating camera info: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
