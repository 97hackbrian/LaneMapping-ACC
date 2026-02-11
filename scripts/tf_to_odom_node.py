#!/usr/bin/env python3
"""
TF to Odom + EKF Node

This node provides a stable, high-rate (100Hz) odometry source by fusing:
1. Low-rate Pose updates from Cartographer (via TF odom->base_link).
2. High-rate Angular Velocity from IMU.

Algorithm (Simplified EKF/Complementary Filter):
- Prediction (100Hz): Integrate IMU gyro for heading. Dead-reckon position using last known velocity.
- Correction (~5-20Hz): When TF updates, correct position/heading towards the TF value, but smooth out jumps.

Publishes:
- /odom_filtered (nav_msgs/Odometry)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import math
import numpy as np

def euler_from_quaternion(q):
    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(t3, t4)

def quaternion_from_euler(roll, pitch, yaw):
    q = Quaternion()
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q

class TfToOdomEKF(Node):
    def __init__(self):
        super().__init__('tf_to_odom_ekf')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('output_topic', '/odom_filtered')
        
        # Filter parameters - Tune these!
        # Alpha controls how much we trust the TF measurement vs our prediction.
        # 0.05 = VERY smooth, trust prediction 95%. 0.5 = trust TF 50%.
        self.declare_parameter('filter_alpha_pos', 0.1) 
        self.declare_parameter('filter_alpha_yaw', 0.05) 

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.output_topic = self.get_parameter('output_topic').value
        self.alpha_pos = self.get_parameter('filter_alpha_pos').value
        self.alpha_yaw = self.get_parameter('filter_alpha_yaw').value

        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State [x, y, theta]
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        # Velocity estimation state
        self.v = 0.0
        self.w = 0.0
        
        # IMU state
        self.gyro_z = 0.0
        self.last_imu_time = self.get_clock().now()

        # Calibration
        self.gyro_bias = 0.0
        self.cal_samples = 200
        self.cal_buffer = []
        self.is_calibrating = True

        # Last TF update timestamp
        self.last_tf_time = Time(seconds=0)
        self.last_tf_pos = (0.0, 0.0)
        self.tf_initialized = False

        # QoS
        qos_imu = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, '/qcar2_imu', self._imu_cb, qos_imu)
        
        self.odom_pub = self.create_publisher(Odometry, self.output_topic, 10)

        # Main Loop (100Hz) - Prediction & Publishing
        self.create_timer(0.01, self._loop)
        
        self.get_logger().info("🚀 TF-to-Odom EKF Started! Calibrating gyro...")

    def _imu_cb(self, msg):
        self.gyro_z = msg.angular_velocity.z
        
        if self.is_calibrating:
            self.cal_buffer.append(self.gyro_z)
            if len(self.cal_buffer) >= self.cal_samples:
                self.gyro_bias = sum(self.cal_buffer) / len(self.cal_buffer)
                self.is_calibrating = False
                self.get_logger().info(f"✅ Gyro Calibrated! Bias: {self.gyro_bias:.5f} rad/s")

    def _loop(self):
        if self.is_calibrating: return

        now = self.get_clock().now()
        dt = (now - self.last_imu_time).nanoseconds * 1e-9
        self.last_imu_time = now
        
        if dt > 0.1 or dt <= 0.0: return # Skip big jumps or zero dt

        # ── 1. Update from TF (Correction Step) ─────────────────────
        try:
            # Get latest available transform (Scan matched pose)
            t = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                Time()) 
            
            px = t.transform.translation.x
            py = t.transform.translation.y
            q = t.transform.rotation
            pth = euler_from_quaternion(q)
            
            tf_time = Time.from_msg(t.header.stamp)
            
            if not self.tf_initialized:
                # First ever TF received -> Jump state to it directly
                self.x, self.y, self.th = px, py, pth
                self.last_tf_time = tf_time
                self.last_tf_pos = (px, py)
                self.tf_initialized = True
                self.get_logger().info("✅ Initial Pose acquired from TF")
            
            else:
                # Calculate time since last TF update we processed
                dt_tf = (tf_time - self.last_tf_time).nanoseconds * 1e-9
                
                # If this is a NEW transform (dt > 0)
                if dt_tf > 0.001:
                    # Estimate velocity from TF position change
                    dist = math.sqrt((px - self.last_tf_pos[0])**2 + (py - self.last_tf_pos[1])**2)
                    v_meas = dist / dt_tf
                    
                    # Direction check (simple dot product with heading)
                    dx = px - self.last_tf_pos[0]
                    dy = py - self.last_tf_pos[1]
                    heading_vec = (math.cos(pth), math.sin(pth))
                    dot = dx*heading_vec[0] + dy*heading_vec[1]
                    if dot < -0.01: v_meas = -v_meas # Limit reverse detection
                    
                    # Update Velocity estimate (Low Pass Filter)
                    # Trust derived velocity slightly, but smooth heavily
                    self.v = 0.9 * self.v + 0.1 * v_meas
                    
                    self.last_tf_time = tf_time
                    self.last_tf_pos = (px, py)

                    # INNOVATION (Error between Measurement and Prediction)
                    def angle_diff(a, b):
                        d = a - b
                        while d > math.pi: d -= 2*math.pi
                        while d < -math.pi: d += 2*math.pi
                        return d

                    err_x = px - self.x
                    err_y = py - self.y
                    err_th = angle_diff(pth, self.th)
                    
                    # Update State (Correction)
                    self.x += self.alpha_pos * err_x
                    self.y += self.alpha_pos * err_y
                    self.th += self.alpha_yaw * err_th

        except Exception:
            pass # TF not ready yet

        # ── 2. Prediction Step (Dead Reckoning) ─────────────────────
        omega = self.gyro_z - self.gyro_bias
        
        # Deadzone for rotation drift
        if abs(omega) < 0.003: omega = 0.0
        
        # Integrate Heading
        self.th += omega * dt
        self.th = math.atan2(math.sin(self.th), math.cos(self.th)) # Normalize

        # Deadzone for velocity (force stop if extremely slow)
        if abs(self.v) < 0.005: self.v = 0.0

        # Integrate Position
        self.x += self.v * math.cos(self.th) * dt
        self.y += self.v * math.sin(self.th) * dt
        
        self.w = omega 

        # ── 3. Publish ──────────────────────────────────────────────
        self._publish_odom(now)


    def _publish_odom(self, now):
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation = quaternion_from_euler(0, 0, self.th)
        
        msg.twist.twist.linear.x = self.v
        msg.twist.twist.angular.z = self.w
        
        # Covariance
        msg.pose.covariance = [
            0.01, 0., 0., 0., 0., 0.,
            0., 0.01, 0., 0., 0., 0.,
            0., 0., 1e6, 0., 0., 0.,
            0., 0., 0., 1e6, 0., 0.,
            0., 0., 0., 0., 1e6, 0.,
            0., 0., 0., 0., 0., 0.05
        ]
        
        self.odom_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TfToOdomEKF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
