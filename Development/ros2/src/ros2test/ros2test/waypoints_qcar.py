"""
Waypoint Publisher - Publishes SDCSRoadMap waypoints in SDCQcar frame.

NO manual transforms here. The static TF SDCQcar -> map (published in the
launch file) handles all coordinate alignment between the virtual road map
and the physical SLAM map automatically.

Architecture:
  - Waypoints are published in SDCQcar frame (the QCar world)
  - TF tree: SDCQcar -> map -> odom -> base_link
  - ROS (AMCL, Nav2) works in map frame as normal
  - VehicleLogic gets robot pose in SDCQcar via TF lookup
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from hal.products.mats import SDCSRoadMap
import numpy as np


class WaypointsQCar(Node):
    def __init__(self):
        super().__init__('waypoint_qcar')

        # Node sequence for SDCSRoadMap path generation
        self.declare_parameters(
            namespace='',
            parameters=[('nodeSequence', [10, 2, 4, 6, 8, 10])]
        )

        self.nodeSequence = self.get_parameter("nodeSequence").value
        self.roadmap = SDCSRoadMap(leftHandTraffic=False, useSmallMap=True)
        self.waypointSequence = self.roadmap.generate_path(self.nodeSequence) * 0.975
        self.waypoints_x = self.waypointSequence[0]
        self.waypoints_y = self.waypointSequence[1]

        # Publishers for visualization and planning (in SDCQcar frame)
        self.vis_path_pub = self.create_publisher(MarkerArray, "/waypoints_viz_qcar", 10)
        self.path_pub = self.create_publisher(Path, "/plan_qcar", 10)

        self.timer = self.create_timer(0.5, self.pub)

        self.get_logger().info("Waypoint publisher initialized - publishing in SDCQcar frame")
        self.get_logger().info(f"Node sequence: {self.nodeSequence}")
        self.get_logger().info(f"Number of waypoints: {len(self.waypoints_x)}")
        self.get_logger().info("NO manual transforms - TF SDCQcar->map handles alignment")

    def pub(self):
        marker_array = MarkerArray()
        path_msg = Path()
        # Publish in SDCQcar frame - TF will convert to map automatically
        path_msg.header.frame_id = "SDCQcar"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for i, (raw_x, raw_y) in enumerate(zip(self.waypoints_x, self.waypoints_y)):
            # Raw SDCSRoadMap coordinates - NO transformation
            x_final = float(raw_x)
            y_final = float(raw_y)

            # --- Visual Markers ---
            marker = Marker()
            marker.header.frame_id = "SDCQcar"
            marker.type = Marker.SPHERE
            marker.id = i
            marker.action = Marker.ADD
            marker.pose.position.x = x_final
            marker.pose.position.y = y_final
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.15
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.lifetime = Duration(sec=1, nanosec=0)
            marker_array.markers.append(marker)

            # --- Robot Path ---
            pose = PoseStamped()
            pose.header.frame_id = "SDCQcar"
            pose.pose.position.x = x_final
            pose.pose.position.y = y_final
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.vis_path_pub.publish(marker_array)
        self.path_pub.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointsQCar()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
