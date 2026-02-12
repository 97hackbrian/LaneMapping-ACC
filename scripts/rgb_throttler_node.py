#!/usr/bin/env python3
"""
RGB Throttler Node

Throttles the RGB image topic to a target frequency (e.g., 6 Hz)
to match the slower masked depth image for better RTAB-Map synchronization.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import time

class RgbThrottlerNode(Node):
    def __init__(self):
        super().__init__('rgb_throttler')
        
        # Parameters
        self.declare_parameter('input_topic', '/camera/color_image')
        self.declare_parameter('output_topic', '/camera/color_image_throttled')
        self.declare_parameter('target_fps', 6.0)
        
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.target_fps = self.get_parameter('target_fps').value
        
        self.min_period = 1.0 / self.target_fps
        self.last_pub_time = 0.0
        
        # Subscriber
        self.sub = self.create_subscription(
            Image,
            self.input_topic,
            self.callback,
            qos_profile=rclpy.qos.qos_profile_sensor_data # Best effort
        )
        
        # Publisher
        self.pub = self.create_publisher(
            Image,
            self.output_topic,
            qos_profile=rclpy.qos.qos_profile_sensor_data
        )
        
        self.get_logger().info(
            f'Throttler started: {self.input_topic} -> {self.output_topic} @ {self.target_fps} Hz'
        )

    def callback(self, msg):
        current_time = time.time()
        if (current_time - self.last_pub_time) >= self.min_period:
            self.pub.publish(msg)
            self.last_pub_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = RgbThrottlerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
