import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class AmclInitialPosePublisher(Node):
    """Publish a startup initial pose once AMCL subscribes to /initialpose."""

    def __init__(self):
        super().__init__('amcl_initial_pose_publisher')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('topic', '/initialpose'),
                ('frame_id', 'map'),
                ('x', 0.0),
                ('y', 0.0),
                ('yaw_deg', 0.0),
                ('covariance_xy', 0.05),
                ('covariance_yaw', 0.05),
                ('check_period_sec', 0.2),
                ('publish_repetitions', 3),
                ('timeout_sec', 30.0),
            ],
        )

        self.topic = str(self.get_parameter('topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.x = float(self.get_parameter('x').value)
        self.y = float(self.get_parameter('y').value)
        self.yaw_deg = float(self.get_parameter('yaw_deg').value)
        self.covariance_xy = float(self.get_parameter('covariance_xy').value)
        self.covariance_yaw = float(self.get_parameter('covariance_yaw').value)
        self.publish_repetitions = max(1, int(self.get_parameter('publish_repetitions').value))
        self.timeout_sec = max(0.0, float(self.get_parameter('timeout_sec').value))
        check_period_sec = max(0.05, float(self.get_parameter('check_period_sec').value))

        self.publisher = self.create_publisher(PoseWithCovarianceStamped, self.topic, 10)
        self.start_time = self.get_clock().now()
        self.publish_count = 0
        self.wait_logged = False
        self.timer = self.create_timer(check_period_sec, self._on_timer)

    def _on_timer(self):
        if self.publish_count >= self.publish_repetitions:
            self.get_logger().info('Initial pose publishing complete; shutting down helper.')
            self.timer.cancel()
            self.destroy_node()
            rclpy.shutdown()
            return

        if self.publisher.get_subscription_count() == 0:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
            if not self.wait_logged:
                self.wait_logged = True
                self.get_logger().info(
                    f"Waiting for AMCL subscriber on {self.topic} before publishing initial pose."
                )
            if self.timeout_sec > 0.0 and elapsed >= self.timeout_sec:
                self.get_logger().warning(
                    f"No subscriber appeared on {self.topic} within {self.timeout_sec:.1f} s; "
                    'stopping initial pose helper.'
                )
                self.timer.cancel()
                self.destroy_node()
                rclpy.shutdown()
            return

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = math.sin(math.radians(self.yaw_deg) / 2.0)
        msg.pose.pose.orientation.w = math.cos(math.radians(self.yaw_deg) / 2.0)
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = self.covariance_xy
        msg.pose.covariance[7] = self.covariance_xy
        msg.pose.covariance[35] = self.covariance_yaw

        self.publisher.publish(msg)
        self.publish_count += 1

        self.get_logger().info(
            f"Published initial pose {self.publish_count}/{self.publish_repetitions}: "
            f"x={self.x:.3f}, y={self.y:.3f}, yaw_deg={self.yaw_deg:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = AmclInitialPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
