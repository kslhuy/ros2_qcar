import math
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformBroadcaster


class QCarDeadReckoningOdom(Node):
    """Publish a lightweight odom -> base_link transform from QCar sensors."""

    def __init__(self):
        super().__init__('qcar_dead_reckoning_odom')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('odom_frame_id', 'odom'),
                ('base_frame_id', 'base_link'),
                ('joint_topic', '/qcar2_joint'),
                ('imu_topic', '/qcar2_imu'),
                ('odom_topic', '/odom'),
                ('publish_tf', True),
                ('publish_rate', 50.0),
                ('encoder_counts_per_rev', 720.0),
                ('wheel_radius', 0.033),
                ('pin_to_spur_ratio', 0.09536679536679536),
                ('linear_velocity_deadband_mps', 0.01),
                ('angular_velocity_deadband_rad_s', 0.02),
                ('yaw_rate_ema_alpha', 0.25),
            ],
        )

        self.odom_frame_id = str(self.get_parameter('odom_frame_id').value)
        self.base_frame_id = str(self.get_parameter('base_frame_id').value)
        self.joint_topic = str(self.get_parameter('joint_topic').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.linear_velocity_deadband_mps = abs(
            float(self.get_parameter('linear_velocity_deadband_mps').value)
        )
        self.angular_velocity_deadband_rad_s = abs(
            float(self.get_parameter('angular_velocity_deadband_rad_s').value)
        )
        self.yaw_rate_ema_alpha = min(
            max(float(self.get_parameter('yaw_rate_ema_alpha').value), 0.0),
            1.0,
        )

        publish_rate = max(float(self.get_parameter('publish_rate').value), 1.0)
        encoder_counts_per_rev = float(self.get_parameter('encoder_counts_per_rev').value)
        wheel_radius = float(self.get_parameter('wheel_radius').value)
        pin_to_spur_ratio = float(self.get_parameter('pin_to_spur_ratio').value)

        self.cps_to_mps = (
            (1.0 / (encoder_counts_per_rev * 4.0))
            * pin_to_spur_ratio
            * 2.0
            * math.pi
            * wheel_radius
        )

        self.linear_velocity = 0.0
        self.yaw_rate = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_update_time: Optional[rclpy.time.Time] = None
        self._joint_logged = False
        self._imu_logged = False

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.create_subscription(JointState, self.joint_topic, self._joint_callback, 20)
        self.create_subscription(Imu, self.imu_topic, self._imu_callback, 20)
        self.timer = self.create_timer(1.0 / publish_rate, self._publish_odometry)

        self.get_logger().info(
            "Dead-reckoning odom enabled: "
            f"{self.odom_frame_id} -> {self.base_frame_id}, "
            f"joint={self.joint_topic}, imu={self.imu_topic}, rate={publish_rate:.1f} Hz"
        )

    def _joint_callback(self, msg: JointState):
        if msg.velocity:
            linear_velocity = float(msg.velocity[0]) * self.cps_to_mps
            if abs(linear_velocity) < self.linear_velocity_deadband_mps:
                linear_velocity = 0.0
            self.linear_velocity = linear_velocity
            if not self._joint_logged:
                self._joint_logged = True
                self.get_logger().info("QCar joint state connected for odometry")

    def _imu_callback(self, msg: Imu):
        yaw_rate = float(msg.angular_velocity.z)
        if abs(yaw_rate) < self.angular_velocity_deadband_rad_s:
            yaw_rate = 0.0

        alpha = self.yaw_rate_ema_alpha
        self.yaw_rate = alpha * yaw_rate + (1.0 - alpha) * self.yaw_rate
        if abs(self.yaw_rate) < self.angular_velocity_deadband_rad_s:
            self.yaw_rate = 0.0

        if not self._imu_logged:
            self._imu_logged = True
            self.get_logger().info("QCar IMU connected for odometry")

    def _publish_odometry(self):
        now = self.get_clock().now()
        if self.last_update_time is None:
            self.last_update_time = now
            return

        dt = (now - self.last_update_time).nanoseconds * 1e-9
        if dt < 0.0:
            self.get_logger().warning("Clock moved backwards; resetting odom integrator")
            self.last_update_time = now
            return
        if dt > 0.5:
            dt = 0.5

        self.last_update_time = now

        if (
            abs(self.linear_velocity) < self.linear_velocity_deadband_mps
            and abs(self.yaw_rate) < self.angular_velocity_deadband_rad_s
        ):
            self.linear_velocity = 0.0
            self.yaw_rate = 0.0

        self.yaw += self.yaw_rate * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        self.x += self.linear_velocity * math.cos(self.yaw) * dt
        self.y += self.linear_velocity * math.sin(self.yaw) * dt

        quat = R.from_euler('xyz', [0.0, 0.0, self.yaw]).as_quat()

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = float(quat[0])
        odom.pose.pose.orientation.y = float(quat[1])
        odom.pose.pose.orientation.z = float(quat[2])
        odom.pose.pose.orientation.w = float(quat[3])
        odom.twist.twist.linear.x = float(self.linear_velocity)
        odom.twist.twist.angular.z = float(self.yaw_rate)

        odom.pose.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
        ]
        odom.twist.covariance = [
            0.2, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.2, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.2,
        ]
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
    node = QCarDeadReckoningOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
