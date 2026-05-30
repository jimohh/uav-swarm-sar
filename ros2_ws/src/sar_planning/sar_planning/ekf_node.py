#!/usr/bin/env python3
"""
ekf_node.py
Extended Kalman Filter for UAV pose estimation.
Fuses IMU (accelerometer + gyroscope) and GPS measurements
to produce a clean, low-latency pose estimate published
as nav_msgs/Odometry on /sar/ekf/odom.

State vector: [x, y, z, vx, vy, vz, roll, pitch, yaw]
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import math


class EKFNode(Node):

    def __init__(self):
        super().__init__('ekf_node')

        # --- Parameters ---
        self.declare_parameter('uav_ns', 'uav0')
        self.declare_parameter('publish_rate', 50.0)  # Hz
        # Process noise
        self.declare_parameter('q_pos', 0.01)
        self.declare_parameter('q_vel', 0.1)
        self.declare_parameter('q_att', 0.001)
        # Measurement noise
        self.declare_parameter('r_gps', 2.5)    # metres std dev
        self.declare_parameter('r_imu', 0.01)   # rad/s std dev

        ns      = self.get_parameter('uav_ns').value
        rate    = self.get_parameter('publish_rate').value
        q_pos   = self.get_parameter('q_pos').value
        q_vel   = self.get_parameter('q_vel').value
        q_att   = self.get_parameter('q_att').value
        r_gps   = self.get_parameter('r_gps').value
        r_imu   = self.get_parameter('r_imu').value

        # --- EKF state initialisation ---
        # State: [x, y, z, vx, vy, vz, roll, pitch, yaw]
        self.n  = 9
        self.x  = np.zeros(self.n)           # state vector
        self.P  = np.eye(self.n) * 1.0       # covariance matrix

        # Process noise covariance Q
        self.Q = np.diag([
            q_pos, q_pos, q_pos,   # position
            q_vel, q_vel, q_vel,   # velocity
            q_att, q_att, q_att    # attitude
        ])

        # GPS measurement noise covariance
        self.R_gps = np.diag([r_gps**2, r_gps**2, r_gps**2])

        # IMU measurement noise covariance
        self.R_imu = np.diag([r_imu**2, r_imu**2, r_imu**2])

        # GPS origin for local frame conversion
        self.gps_origin = None

        # Timing
        self.last_time  = None
        self.imu_data   = None
        self.initialized = False

        # --- Subscribers ---
        self.create_subscription(
            Imu,
            f'/{ns}/mavros/imu/data',
            self._imu_callback, 50)

        self.create_subscription(
            NavSatFix,
            f'/{ns}/mavros/global_position/global',
            self._gps_callback, 10)

        self.create_subscription(
            Odometry,
            f'/{ns}/mavros/local_position/odom',
            self._odom_callback, 50)

        # --- Publishers ---
        self.pub_ekf = self.create_publisher(
            Odometry, '/sar/ekf/odom', 50)

        # Prediction timer
        self.timer = self.create_timer(
            1.0 / rate, self._predict_and_publish)

        self.get_logger().info(
            f'EKFNode started for {ns} at {rate} Hz')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _imu_callback(self, msg):
        """Store latest IMU data for prediction step."""
        self.imu_data = msg
        if not self.initialized:
            # Initialise attitude from IMU quaternion
            q = msg.orientation
            self.x[6], self.x[7], self.x[8] = \
                self._quat_to_euler(q.w, q.x, q.y, q.z)
            self.initialized = True

    def _gps_callback(self, msg):
        """GPS measurement update step."""
        if msg.status.status < 0:
            return  # no fix

        if self.gps_origin is None:
            self.gps_origin = (msg.latitude, msg.longitude, msg.altitude)
            self.get_logger().info(
                f'GPS origin set: {self.gps_origin[0]:.6f}, '
                f'{self.gps_origin[1]:.6f}')
            return

        # Convert GPS to local NED metres
        z_meas = np.array(self._gps_to_local(
            msg.latitude, msg.longitude, msg.altitude))

        # EKF update — GPS measures position only (indices 0,1,2)
        H = np.zeros((3, self.n))
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[2, 2] = 1.0

        self._measurement_update(z_meas, H, self.R_gps)

    def _odom_callback(self, msg):
        """
        Use MAVROS local odometry as a direct state initialiser.
        This gives EKF a warm start from PX4's own estimator,
        which the EKF then refines with its own fusion.
        """
        if not self.initialized:
            p = msg.pose.pose.position
            v = msg.twist.twist.linear
            self.x[0] = p.x
            self.x[1] = p.y
            self.x[2] = p.z
            self.x[3] = v.x
            self.x[4] = v.y
            self.x[5] = v.z
            self.initialized = True

    # ------------------------------------------------------------------
    # EKF predict step
    # ------------------------------------------------------------------
    def _predict_and_publish(self):
        if not self.initialized:
            return

        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0 or dt > 0.5:
            return

        # State transition — constant velocity model
        # x_new = F * x + B * u
        F = np.eye(self.n)
        F[0, 3] = dt   # x  += vx * dt
        F[1, 4] = dt   # y  += vy * dt
        F[2, 5] = dt   # z  += vz * dt

        # IMU angular rate input
        if self.imu_data is not None:
            av = self.imu_data.angular_velocity
            self.x[6] += av.x * dt   # roll
            self.x[7] += av.y * dt   # pitch
            self.x[8] += av.z * dt   # yaw

            # Linear acceleration input (body frame → world frame)
            la = self.imu_data.linear_acceleration
            cr, sr = math.cos(self.x[6]), math.sin(self.x[6])
            cp, sp = math.cos(self.x[7]), math.sin(self.x[7])
            cy, sy = math.cos(self.x[8]), math.sin(self.x[8])

            ax_w = (cp*cy)*la.x + (sr*sp*cy - cr*sy)*la.y + \
                   (cr*sp*cy + sr*sy)*la.z
            ay_w = (cp*sy)*la.x + (sr*sp*sy + cr*cy)*la.y + \
                   (cr*sp*sy - sr*cy)*la.z
            az_w = (-sp)*la.x + (sr*cp)*la.y + (cr*cp)*la.z - 9.81

            self.x[3] += ax_w * dt
            self.x[4] += ay_w * dt
            self.x[5] += az_w * dt

        # Covariance prediction: P = F*P*F^T + Q
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

        self._publish_state()

    # ------------------------------------------------------------------
    # EKF measurement update
    # ------------------------------------------------------------------
    def _measurement_update(self, z, H, R):
        """Generic EKF update step."""
        y = z - H @ self.x                        # innovation
        S = H @ self.P @ H.T + R                  # innovation covariance
        K = self.P @ H.T @ np.linalg.inv(S)       # Kalman gain
        self.x = self.x + K @ y                   # state update
        self.P = (np.eye(self.n) - K @ H) @ self.P  # covariance update

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------
    def _publish_state(self):
        msg = Odometry()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id  = 'base_link'

        msg.pose.pose.position.x = self.x[0]
        msg.pose.pose.position.y = self.x[1]
        msg.pose.pose.position.z = self.x[2]

        # Convert euler back to quaternion for message
        w, qx, qy, qz = self._euler_to_quat(
            self.x[6], self.x[7], self.x[8])
        msg.pose.pose.orientation.w = w
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz

        msg.twist.twist.linear.x = self.x[3]
        msg.twist.twist.linear.y = self.x[4]
        msg.twist.twist.linear.z = self.x[5]

        # Flatten 3x3 position covariance block into 6x6 pose covariance
        for i in range(3):
            for j in range(3):
                msg.pose.covariance[i*6 + j] = self.P[i, j]

        self.pub_ekf.publish(msg)

    # ------------------------------------------------------------------
    # Utility
    # ------------------------------------------------------------------
    def _gps_to_local(self, lat, lon, alt):
        """Convert GPS coordinates to local NED metres from origin."""
        R_earth = 6371000.0
        lat0, lon0, alt0 = self.gps_origin
        x = R_earth * math.radians(lat - lat0)
        y = R_earth * math.cos(math.radians(lat0)) * \
            math.radians(lon - lon0)
        z = alt - alt0
        return x, y, z

    def _quat_to_euler(self, w, x, y, z):
        """Quaternion to roll/pitch/yaw."""
        roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
        yaw   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        return roll, pitch, yaw

    def _euler_to_quat(self, roll, pitch, yaw):
        """Roll/pitch/yaw to quaternion."""
        cr, sr = math.cos(roll/2),  math.sin(roll/2)
        cp, sp = math.cos(pitch/2), math.sin(pitch/2)
        cy, sy = math.cos(yaw/2),   math.sin(yaw/2)
        w =  cr*cp*cy + sr*sp*sy
        x =  sr*cp*cy - cr*sp*sy
        y =  cr*sp*cy + sr*sp*sy
        z =  cr*cp*sy - sr*sp*cy
        return w, x, y, z


def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()