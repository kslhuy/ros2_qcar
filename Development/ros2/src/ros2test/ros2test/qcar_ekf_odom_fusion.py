import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformBroadcaster


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class QCarEkfOdomFusion(Node):
    """Fuse wheel/IMU and RF2O odometry into a single odom -> base_link estimate."""

    def __init__(self):
        super().__init__('qcar_ekf_odom_fusion')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('odom_frame_id', 'odom'),
                ('base_frame_id', 'base_link'),
                ('wheel_odom_topic', '/odom/wheel'),
                ('rf2o_odom_topic', '/odom/rf2o'),
                ('odom_topic', '/odom'),
                ('publish_tf', True),
                ('publish_rate', 40.0),
                ('stale_data_timeout', 0.5),
                ('process_noise_diag', [0.02, 0.02, 0.01, 0.08, 0.04]),
                ('initial_covariance_diag', [1.0, 1.0, 0.3, 0.5, 0.3]),
                ('wheel_twist_noise_diag', [0.08, 0.05]),
                ('rf2o_pose_noise_diag', [0.03, 0.03, 0.02]),
                ('rf2o_twist_noise_diag', [0.06, 0.04]),
            ],
        )

        self.odom_frame_id = str(self.get_parameter('odom_frame_id').value)
        self.base_frame_id = str(self.get_parameter('base_frame_id').value)
        self.wheel_odom_topic = str(self.get_parameter('wheel_odom_topic').value)
        self.rf2o_odom_topic = str(self.get_parameter('rf2o_odom_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.stale_data_timeout = float(self.get_parameter('stale_data_timeout').value)

        self.process_noise_diag = np.array(
            self.get_parameter('process_noise_diag').value,
            dtype=float,
        )
        self.initial_covariance_diag = np.array(
            self.get_parameter('initial_covariance_diag').value,
            dtype=float,
        )
        self.wheel_twist_noise_diag = np.array(
            self.get_parameter('wheel_twist_noise_diag').value,
            dtype=float,
        )
        self.rf2o_pose_noise_diag = np.array(
            self.get_parameter('rf2o_pose_noise_diag').value,
            dtype=float,
        )
        self.rf2o_twist_noise_diag = np.array(
            self.get_parameter('rf2o_twist_noise_diag').value,
            dtype=float,
        )

        self.state = np.zeros(5, dtype=float)  # x, y, yaw, vx, wz
        self.covariance = np.diag(self.initial_covariance_diag)
        self.last_state_time: Optional[Time] = None
        self.last_wheel_time: Optional[Time] = None
        self.last_rf2o_time: Optional[Time] = None
        self._wheel_logged = False
        self._rf2o_logged = False

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.create_subscription(Odometry, self.wheel_odom_topic, self._wheel_callback, 20)
        self.create_subscription(Odometry, self.rf2o_odom_topic, self._rf2o_callback, 20)

        publish_rate = max(float(self.get_parameter('publish_rate').value), 1.0)
        self.timer = self.create_timer(1.0 / publish_rate, self._publish_odometry)

        self.get_logger().info(
            "EKF odom fusion enabled: "
            f"wheel={self.wheel_odom_topic}, rf2o={self.rf2o_odom_topic}, out={self.odom_topic}"
        )

    def _predict_to(self, stamp: Time):
        if self.last_state_time is None:
            self.last_state_time = stamp
            return

        dt = (stamp - self.last_state_time).nanoseconds * 1e-9
        if dt <= 0.0:
            self.last_state_time = stamp
            return
        dt = min(dt, 0.2)

        x, y, yaw, vx, wz = self.state
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        self.state[0] = x + vx * cos_yaw * dt
        self.state[1] = y + vx * sin_yaw * dt
        self.state[2] = _normalize_angle(yaw + wz * dt)

        jacobian = np.eye(5, dtype=float)
        jacobian[0, 2] = -vx * sin_yaw * dt
        jacobian[0, 3] = cos_yaw * dt
        jacobian[1, 2] = vx * cos_yaw * dt
        jacobian[1, 3] = sin_yaw * dt
        jacobian[2, 4] = dt

        process_noise = np.diag(self.process_noise_diag * max(dt, 1e-3))
        self.covariance = jacobian @ self.covariance @ jacobian.T + process_noise
        self.last_state_time = stamp

    def _update(self, measurement: np.ndarray, h_matrix: np.ndarray, innovation: np.ndarray, noise: np.ndarray):
        s_matrix = h_matrix @ self.covariance @ h_matrix.T + noise
        kalman_gain = self.covariance @ h_matrix.T @ np.linalg.inv(s_matrix)
        self.state = self.state + kalman_gain @ innovation
        self.state[2] = _normalize_angle(self.state[2])
        identity = np.eye(self.covariance.shape[0], dtype=float)
        self.covariance = (
            (identity - kalman_gain @ h_matrix)
            @ self.covariance
            @ (identity - kalman_gain @ h_matrix).T
            + kalman_gain @ noise @ kalman_gain.T
        )

    def _wheel_callback(self, msg: Odometry):
        stamp = Time.from_msg(msg.header.stamp)
        self._predict_to(stamp)

        measurement = np.array(
            [
                float(msg.twist.twist.linear.x),
                float(msg.twist.twist.angular.z),
            ],
            dtype=float,
        )
        expected = np.array([self.state[3], self.state[4]], dtype=float)
        innovation = measurement - expected

        wheel_cov = msg.twist.covariance
        noise = np.diag([
            float(wheel_cov[0]) if len(wheel_cov) >= 1 and wheel_cov[0] > 0.0 else self.wheel_twist_noise_diag[0],
            float(wheel_cov[35]) if len(wheel_cov) >= 36 and wheel_cov[35] > 0.0 else self.wheel_twist_noise_diag[1],
        ])

        h_matrix = np.array(
            [
                [0.0, 0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        self._update(measurement, h_matrix, innovation, noise)
        self.last_wheel_time = stamp

        if not self._wheel_logged:
            self._wheel_logged = True
            self.get_logger().info("Wheel/IMU odom connected for EKF fusion")

    def _rf2o_callback(self, msg: Odometry):
        stamp = Time.from_msg(msg.header.stamp)
        self._predict_to(stamp)

        quat = msg.pose.pose.orientation
        yaw = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler('xyz')[2]
        measurement = np.array(
            [
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
                float(yaw),
                float(msg.twist.twist.linear.x),
                float(msg.twist.twist.angular.z),
            ],
            dtype=float,
        )

        expected = np.array(
            [self.state[0], self.state[1], self.state[2], self.state[3], self.state[4]],
            dtype=float,
        )
        innovation = measurement - expected
        innovation[2] = _normalize_angle(innovation[2])

        pose_cov = msg.pose.covariance
        twist_cov = msg.twist.covariance
        noise = np.diag([
            float(pose_cov[0]) if len(pose_cov) >= 1 and pose_cov[0] > 0.0 else self.rf2o_pose_noise_diag[0],
            float(pose_cov[7]) if len(pose_cov) >= 8 and pose_cov[7] > 0.0 else self.rf2o_pose_noise_diag[1],
            float(pose_cov[35]) if len(pose_cov) >= 36 and pose_cov[35] > 0.0 else self.rf2o_pose_noise_diag[2],
            float(twist_cov[0]) if len(twist_cov) >= 1 and twist_cov[0] > 0.0 else self.rf2o_twist_noise_diag[0],
            float(twist_cov[35]) if len(twist_cov) >= 36 and twist_cov[35] > 0.0 else self.rf2o_twist_noise_diag[1],
        ])

        h_matrix = np.eye(5, dtype=float)
        self._update(measurement, h_matrix, innovation, noise)
        self.last_rf2o_time = stamp

        if not self._rf2o_logged:
            self._rf2o_logged = True
            self.get_logger().info("RF2O odom connected for EKF fusion")

    def _publish_odometry(self):
        now = self.get_clock().now()

        if self.last_state_time is not None:
            self._predict_to(now)

        if self.last_wheel_time is None and self.last_rf2o_time is None:
            return

        if self.last_wheel_time is not None:
            wheel_age = (now - self.last_wheel_time).nanoseconds * 1e-9
            if wheel_age > self.stale_data_timeout:
                self.get_logger().warning("Wheel/IMU odom input is stale")

        if self.last_rf2o_time is not None:
            rf2o_age = (now - self.last_rf2o_time).nanoseconds * 1e-9
            if rf2o_age > self.stale_data_timeout:
                self.get_logger().warning("RF2O odom input is stale")

        quat = R.from_euler('xyz', [0.0, 0.0, self.state[2]]).as_quat()

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = float(self.state[0])
        odom.pose.pose.position.y = float(self.state[1])
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = float(quat[0])
        odom.pose.pose.orientation.y = float(quat[1])
        odom.pose.pose.orientation.z = float(quat[2])
        odom.pose.pose.orientation.w = float(quat[3])
        odom.twist.twist.linear.x = float(self.state[3])
        odom.twist.twist.angular.z = float(self.state[4])

        odom.pose.covariance = [99999.0] * 36
        odom.twist.covariance = [99999.0] * 36
        odom.pose.covariance[0] = float(self.covariance[0, 0])
        odom.pose.covariance[7] = float(self.covariance[1, 1])
        odom.pose.covariance[35] = float(self.covariance[2, 2])
        odom.twist.covariance[0] = float(self.covariance[3, 3])
        odom.twist.covariance[35] = float(self.covariance[4, 4])
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = odom.header.stamp
            transform.header.frame_id = self.odom_frame_id
            transform.child_frame_id = self.base_frame_id
            transform.transform.translation.x = odom.pose.pose.position.x
            transform.transform.translation.y = odom.pose.pose.position.y
            transform.transform.translation.z = 0.0
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = QCarEkfOdomFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
