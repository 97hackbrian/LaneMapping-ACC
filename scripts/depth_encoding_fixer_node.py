#!/usr/bin/env python3
"""
Depth Encoding Fixer Node

Converts depth images from 'mono16' encoding to '32FC1' (float32, meters).
RTAB-Map's rgbd_odometry and rtabmap SLAM expect depth images in '16UC1' (mm)
or '32FC1' (meters) format. The QCar2 simulator publishes 'mono16' which
is misinterpreted, causing SIGFPE crashes.

This node:
1. Reads mono16 depth data (assumed millimeters as uint16)
2. Converts to float32 in meters
3. Sets invalid pixels (depth=0) to NaN (RTAB-Map ignores NaN)
4. Republishes as 32FC1
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
import numpy as np
import struct


class DepthEncodingFixerNode(Node):
    """Converts depth images from mono16 to 32FC1 with NaN filtering."""

    def __init__(self):
        super().__init__('depth_encoding_fixer')

        # Parameters
        self.declare_parameter('input_topic', '/camera/depth_image')
        self.declare_parameter('output_topic', '/camera/depth_image_fixed')
        self.declare_parameter('depth_scale', 0.001)  # mono16 mm -> meters

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.depth_scale = self.get_parameter('depth_scale').value

        # QoS: match the source (Best Effort)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub = self.create_subscription(
            Image, input_topic, self.callback, qos)

        self.pub = self.create_publisher(Image, output_topic, qos)

        self.logged = False
        self.get_logger().info(
            f'Depth Encoding Fixer: {input_topic} (mono16) -> '
            f'{output_topic} (32FC1, scale={self.depth_scale})')

    def callback(self, msg: Image):
        try:
            # Convert raw bytes to uint16 numpy array
            depth_u16 = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                (msg.height, msg.width))

            # Log depth range once for debugging
            if not self.logged:
                valid = depth_u16[depth_u16 > 0]
                if len(valid) > 0:
                    self.get_logger().info(
                        f'Depth stats: min={valid.min()}, max={valid.max()}, '
                        f'mean={valid.mean():.1f}, zeros={np.sum(depth_u16==0)}/{depth_u16.size}')
                self.logged = True

            # Convert to float32 meters
            depth_f32 = depth_u16.astype(np.float32) * self.depth_scale

            # Set invalid depth (0) to NaN - RTAB-Map ignores NaN gracefully
            depth_f32[depth_u16 == 0] = float('nan')

            # Build output message
            out = Image()
            out.header = msg.header
            out.height = msg.height
            out.width = msg.width
            out.encoding = '32FC1'
            out.is_bigendian = 0
            out.step = msg.width * 4  # 4 bytes per float32
            out.data = depth_f32.tobytes()

            self.pub.publish(out)

        except Exception as e:
            self.get_logger().error(f'Error converting depth: {e}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = DepthEncodingFixerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
