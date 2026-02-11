#!/usr/bin/env python3
"""
QCar2 Robust Odometry Node (IMU Heading + Encoder Speed)

This node prioritizes the IMU Gyroscope (Z-axis) for heading estimation,
as it is physically more accurate than the commanded steering angle (Ackermann model)
which suffers from mechanical backlash and bias.

Startup Calibration:
- The node samples the IMU for ~2 seconds at startup to determine the gyroscope bias.
- During this phase, the robot MUST be stationary.

Odometry Calculation:
- Linear Velocity (v): Encoder-derived speed (m/s).
- Angular Velocity (ω): (Gyro_Z - Bias)
  *Deadzone applied near zero speed to prevent drift when stopped.

Published transforms:
- /odom topic
- odom -> base_link TF

"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from typing import Optional

from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from qcar2_interfaces.msg import MotorCommands

import tf2_ros

# ── QCar2 Physical Constants ────────────────────────────────────────
WHEEL_RADIUS = 0.033  # meters
ENCODER_CPR = 720
QUAD_MULT = 4
GEAR_RATIO = (13.0 * 19.0) / (70.0 * 30.0)

# Speed Conversion Factor: (counts/s) -> (m/s)
SPEED_FACTOR = (1.0 / (ENCODER_CPR * QUAD_MULT)) * GEAR_RATIO * (2.0 * math.pi * WHEEL_RADIUS)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class QCar2RobustOdom(Node):
    def __init__(self):
        super().__init__('joint_to_odom')

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('calibration_samples', 200) # ~2 seconds at 100Hz

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.calibration_samples = self.get_parameter('calibration_samples').value

        # ── State ───────────────────────────────────────────────────
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_stamp: Optional[rclpy.time.Time] = None

        # ── IMU Calibration State ───────────────────────────────────
        self.is_calibrating = True
        self.gyro_bias = 0.0
        self.calibration_buffer = []
        self.latest_gyro_z = 0.0

        # ── ROS Infrastructure ──────────────────────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.create_subscription(
            JointState,
            '/qcar2_joint',
            self._joint_callback,
            qos
        )
        self.create_subscription(
            Imu,
            '/qcar2_imu',
            self._imu_callback,
            qos
        )

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self) if self.publish_tf else None

        self.get_logger().info("⏳ Starting Gyro Calibration... KEEP ROBOT STATIONARY.")

    def _imu_callback(self, msg: Imu):
        """Handle IMU data for calibration and running state."""
        raw_yaw_rate = msg.angular_velocity.z

        if self.is_calibrating:
            self.calibration_buffer.append(raw_yaw_rate)
            if len(self.calibration_buffer) >= self.calibration_samples:
                # Finish calibration
                self.gyro_bias = sum(self.calibration_buffer) / len(self.calibration_buffer)
                self.is_calibrating = False
                self.get_logger().info(f"✅ Calibration DONE. Gyro Bias: {self.gyro_bias:.6f} rad/s")
        else:
            # Store latest gyro reading for the main loop
            self.latest_gyro_z = raw_yaw_rate

    def _joint_callback(self, msg: JointState):
        """Main odometry calculation loop driven by encoder updates."""
        # Wait for calibration to finish
        if self.is_calibrating:
            return

        # Check for valid message content
        if not msg.velocity:
            return

        current_time = rclpy.time.Time.from_msg(msg.header.stamp)

        # Initialize timestamp on first callback
        if self.last_stamp is None:
            self.last_stamp = current_time
            return

        # Calculate dt
        dt_nano = (current_time - self.last_stamp).nanoseconds
        dt = dt_nano * 1e-9
        self.last_stamp = current_time

        if dt <= 0.0:
            return

        # ── 1. Calculate Linear Velocity (v) ────────────────────────
        # Use motor velocity (index 0 usually corresponds to drive motor)
        raw_velocity_counts = msg.velocity[0]
        v_linear = raw_velocity_counts * SPEED_FACTOR

        # Deadzone check (Stationary)
        if abs(v_linear) < 0.005: # 5 mm/s deadzone
            v_linear = 0.0
            omega = 0.0 # Force zero rotation if stopped
            
            # Bonus: Adaptive Bias Correction while stopped
            # Slowly nudge bias towards current reading to track thermal drift
            self.gyro_bias += 0.001 * (self.latest_gyro_z - self.gyro_bias)
            
        else:
            # ── 2. Calculate Angular Velocity (ω) ───────────────────────
            # Use Calibrated Gyro
            omega = self.latest_gyro_z - self.gyro_bias

        # ── 3. Integrate Pose (Runge-Kutta 2 / Midpoint) ────────────
        delta_theta = omega * dt
        half_delta_theta = delta_theta * 0.5
        
        # Update orientation
        self.theta += delta_theta
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Update position
        self.x += v_linear * math.cos(self.theta - half_delta_theta) * dt
        self.y += v_linear * math.sin(self.theta - half_delta_theta) * dt

        # ── 4. Publish Odometry & TF ────────────────────────────────
        self._publish_odometry(msg.header.stamp, v_linear, omega)

    def _publish_odometry(self, stamp, v_linear, omega):
        odom_quat = yaw_to_quaternion(self.theta)

        # Publish /odom message
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # Pose
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = odom_quat

        # Twist
        odom_msg.twist.twist.linear.x = v_linear
        odom_msg.twist.twist.angular.z = omega

        # Covariance
        odom_msg.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]
        odom_msg.twist.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]

        self.odom_pub.publish(odom_msg)

        # Publish TF
        if self.tf_broadcaster:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = odom_quat
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = QCar2RobustOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
