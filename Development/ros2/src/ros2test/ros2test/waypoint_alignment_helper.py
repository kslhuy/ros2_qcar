#!/usr/bin/env python3
"""
Interactive Waypoint Alignment Helper — CALIBRATION TOOL ONLY

This tool helps you FIND the static TF transform SDCQcar → map
by visually aligning SDCSRoadMap waypoints with your physical SLAM map.

Once you have the values, press 'c' to get the static_transform_publisher
command. Paste that into your launch file. Then STOP this tool forever.

Architecture reminder:
  - This tool applies the transform NUMERICALLY (for visual calibration)
  - The final system uses TF (no numerical transforms in nodes)
  - Waypoints are published raw in SDCQcar frame at runtime
  - TF SDCQcar→map handles all alignment automatically

Usage:
1. Launch your map (AMCL or map_server) so /map is visible
2. Run: ros2 run ros2test waypoint_alignment_helper
3. Open RViz, Fixed Frame = 'map', add /map and /waypoints_test
4. Use keyboard to adjust until waypoints align with physical roads
5. Press 'c' to get the static TF command for your launch file
6. Stop this tool — it is NOT needed at runtime
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration
from hal.products.mats import SDCSRoadMap
import numpy as np
import sys
import select
import tty
import termios


class WaypointAlignmentHelper(Node):
    def __init__(self):
        super().__init__('waypoint_alignment_helper')

        # Accept launch-style TF parameters and convert to calibration format.
        # TF inputs represent SDCQcar -> map static transform.
        self.declare_parameter('sdc_map_x', -1.7000)
        self.declare_parameter('sdc_map_y', 0.1000)
        self.declare_parameter('sdc_map_z', 0.0)
        self.declare_parameter('sdc_map_yaw', 1.9635)
        self.declare_parameter('sdc_map_pitch', 0.0)
        self.declare_parameter('sdc_map_roll', 0.0)

        sdc_map_x = float(self.get_parameter('sdc_map_x').value)
        sdc_map_y = float(self.get_parameter('sdc_map_y').value)
        sdc_map_z = float(self.get_parameter('sdc_map_z').value)
        sdc_map_yaw = float(self.get_parameter('sdc_map_yaw').value)
        sdc_map_pitch = float(self.get_parameter('sdc_map_pitch').value)
        sdc_map_roll = float(self.get_parameter('sdc_map_roll').value)

        # Current transformation parameters (visual calibration convention)
        self.translation_x = -sdc_map_x
        self.translation_y = -sdc_map_y
        self.rotation_deg = -np.degrees(sdc_map_yaw)
        self.scale = 1.0

        # Keep the seeded pose for reset.
        self.initial_translation_x = self.translation_x
        self.initial_translation_y = self.translation_y
        self.initial_rotation_deg = self.rotation_deg
        self.initial_scale = self.scale

        if abs(sdc_map_z) > 1e-6 or abs(sdc_map_pitch) > 1e-6 or abs(sdc_map_roll) > 1e-6:
            self.get_logger().warn(
                'sdc_map_z/pitch/roll are accepted for compatibility but ignored by 2D calibration.'
            )

        # Adjustable step sizes (can be tuned live)
        self.translation_step = 0.1
        self.rotation_step = 1.0
        self.scale_step = 0.05
        
        # SDCSRoadMap setup
        self.nodeSequence = [10, 2, 4, 6, 8, 10]
        self.roadmap = SDCSRoadMap(leftHandTraffic=False, useSmallMap=True)
        self.waypointSequence = self.roadmap.generate_path(self.nodeSequence)
        self.waypoints_x = self.waypointSequence[0]
        self.waypoints_y = self.waypointSequence[1]
        
        # Publishers
        self.vis_pub = self.create_publisher(MarkerArray, "/waypoints_test", 10)
        self.path_pub = self.create_publisher(Path, "/path_test", 10)
        
        # Timer for continuous publishing
        self.timer = self.create_timer(0.5, self.publish_waypoints)
        
        self.get_logger().info("="*70)
        self.get_logger().info("Waypoint Alignment Helper Started")
        self.get_logger().info("="*70)
        self.print_instructions()
        self.print_current_params()
        
    def print_instructions(self):
        print("\n" + "="*70)
        print("INTERACTIVE CONTROLS:")
        print("="*70)
        print("Startup params (same names as launch file):")
        print("  --ros-args -p sdc_map_x:=... -p sdc_map_y:=... -p sdc_map_yaw:=...")
        print()
        print("Translation (move waypoints):")
        print("  w/s : Move Y +/- translation_step")
        print("  a/d : Move X -/+ translation_step")
        print("  W/S : Move Y +/- (translation_step × 5)")
        print("  A/D : Move X -/+ (translation_step × 5)")
        print()
        print("Rotation (rotate waypoints):")
        print("  q/e : Rotate -/+ rotation_step")
        print("  Q/E : Rotate -/+ (rotation_step × 3)")
        print()
        print("Scale (resize waypoints):")
        print("  z/x : Scale -/+ scale_step")
        print("  Z/X : Scale -/+ (scale_step × 2)")
        print()
        print("Adjust step intensity live:")
        print("  t/g : Translation step -/+ (0.01m)")
        print("  y/k : Rotation step -/+ (0.5°)")
        print("  u/j : Scale step -/+ (0.01)")
        print()
        print("Utility:")
        print("  r   : Reset to defaults")
        print("  p   : Print current parameters")
        print("  h   : Show this help panel")
        print("  c   : Copy parameters for launch file")
        print("  ESC : Quit")
        print("="*70)
        print("\nOpen RViz and add:")
        print("  1. /map topic (your physical SLAM map)")
        print("  2. /waypoints_test (MarkerArray - cyan spheres)")
        print("  3. Fixed Frame = 'map'")
        print()
        print("This is a CALIBRATION TOOL. When aligned, press 'c' to get")
        print("the static TF command. Then stop this tool forever.")
        print("="*70 + "\n")
    
    def print_current_params(self):
        print(f"\n[Current Parameters]")
        print(f"  translation_offset: [{self.translation_x:.3f}, {self.translation_y:.3f}]")
        print(f"  rotation_offset: {self.rotation_deg:.1f}°")
        print(f"  scale: {self.scale:.3f}")
        print(f"  translation_step: {self.translation_step:.2f} m")
        print(f"  rotation_step: {self.rotation_step:.1f}°")
        print(f"  scale_step: {self.scale_step:.2f}")
        print()
    
    def print_launch_params(self):
        # Convert calibration params → static TF args
        #
        # Math: The calibration tool applies p_map = (p_sdc + t) @ R
        #       where R uses angle_rad = -rotation_deg * pi/180
        #
        # TF convention: static_transform_publisher x y z yaw pitch roll parent child
        #   A point p_parent expressed in child frame: p_child = R(yaw)^T @ (p_parent - [x,y,z])
        #
        # Matching the two:
        #   yaw_tf  = -rotation_deg * pi / 180    (same sign ✓)
        #   x_tf    = -translation_x               (NEGATED!)
        #   y_tf    = -translation_y               (NEGATED!)
        #
        yaw_rad = -self.rotation_deg * np.pi / 180.0
        tf_x = -self.translation_x
        tf_y = -self.translation_y
        
        print("\n" + "="*70)
        print("CALIBRATION RESULT — SDCQcar → map static TF")
        print("="*70)
        
        if abs(self.scale - 1.0) > 0.01:
            print(f"\n⚠️  Scale = {self.scale:.3f} (not 1.0)")
            print("   TF does not support scale. Adjust the SDCSRoadMap factor instead.")
            print(f"   In waypoints_qcar.py, change * 0.975 to * {0.975 * self.scale:.4f}")
        
        print(f"\nCalibration values (visual):")
        print(f"  translation: [{self.translation_x:.4f}, {self.translation_y:.4f}]")
        print(f"  rotation:    {self.rotation_deg:.1f}°")
        print(f"  scale:       {self.scale:.3f}")
        print(f"\nConverted to TF (sign-corrected):")
        print(f"  x={tf_x:.4f}, y={tf_y:.4f}, yaw={yaw_rad:.4f} rad")
        
        print("\n--- PASTE INTO LAUNCH FILE ---")
        print("Node(")
        print("    package='tf2_ros',")
        print("    executable='static_transform_publisher',")
        print("    name='sdcqcar_to_map',")
        print(f"    arguments=['{tf_x:.4f}', '{tf_y:.4f}', '0',")
        print(f"               '{yaw_rad:.4f}', '0', '0', 'SDCQcar', 'map'],")
        print("    output='screen'")
        print("),")
        
        print("\n--- TERMINAL TEST COMMAND ---")
        print(f"ros2 run tf2_ros static_transform_publisher \\")
        print(f"  {tf_x:.4f} {tf_y:.4f} 0 \\")
        print(f"  {yaw_rad:.4f} 0 0 \\")
        print(f"  SDCQcar map")
        
        print("\n" + "="*70 + "\n")
    
    def reset_params(self):
        self.translation_x = self.initial_translation_x
        self.translation_y = self.initial_translation_y
        self.rotation_deg = self.initial_rotation_deg
        self.scale = self.initial_scale
        self.translation_step = 0.1
        self.rotation_step = 5.0
        self.scale_step = 0.05
        self.get_logger().info("Parameters reset to startup values")
        self.print_current_params()
    
    def publish_waypoints(self):
        """Publish transformed waypoints in map frame for visual calibration.
        
        NOTE: This numerical transform is ONLY for calibration visualization.
        At runtime, waypoints are published raw in SDCQcar and TF handles alignment.
        """
        marker_array = MarkerArray()
        path_msg = Path()
        path_msg.header.frame_id = "map"  # Visualize in map frame during calibration
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for i, (raw_x, raw_y) in enumerate(zip(self.waypoints_x, self.waypoints_y)):
            # Apply transform numerically (calibration only!)
            # 1. Scale
            wp_scaled = np.array([raw_x * self.scale, raw_y * self.scale])
            
            # 2. Translate
            t = np.array([self.translation_x, self.translation_y])
            wp_translated = wp_scaled + t
            
            # 3. Rotate (negative angle for coordinate alignment)
            angle_rad = -self.rotation_deg * np.pi / 180.0
            R_matrix = np.array([
                [np.cos(angle_rad), -np.sin(angle_rad)],
                [np.sin(angle_rad),  np.cos(angle_rad)]
            ])
            wp_final = wp_translated @ R_matrix
            
            x_final = wp_final[0]
            y_final = wp_final[1]
            
            # Marker
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = Marker.SPHERE
            marker.id = i
            marker.action = Marker.ADD
            marker.pose.position.x = x_final
            marker.pose.position.y = y_final
            marker.pose.position.z = 0.1
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.lifetime = Duration(sec=1, nanosec=0)
            marker_array.markers.append(marker)
            
            # Path
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x_final
            pose.pose.position.y = y_final
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.vis_pub.publish(marker_array)
        self.path_pub.publish(path_msg)


def get_key():
    """Get single keypress (non-blocking)"""
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


def main(args=None):
    rclpy.init(args=args)
    node = WaypointAlignmentHelper()
    
    # Set terminal to raw mode for key capture
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            
            key = get_key()
            if key:
                changed = True
                coarse_translation = node.translation_step * 5.0
                coarse_rotation = node.rotation_step * 3.0
                coarse_scale = node.scale_step * 2.0
                
                # Translation
                if key == 'w':
                    node.translation_y += node.translation_step
                elif key == 's':
                    node.translation_y -= node.translation_step
                elif key == 'a':
                    node.translation_x -= node.translation_step
                elif key == 'd':
                    node.translation_x += node.translation_step
                elif key == 'W':
                    node.translation_y += coarse_translation
                elif key == 'S':
                    node.translation_y -= coarse_translation
                elif key == 'A':
                    node.translation_x -= coarse_translation
                elif key == 'D':
                    node.translation_x += coarse_translation
                
                # Rotation
                elif key == 'q':
                    node.rotation_deg -= node.rotation_step
                elif key == 'e':
                    node.rotation_deg += node.rotation_step
                elif key == 'Q':
                    node.rotation_deg -= coarse_rotation
                elif key == 'E':
                    node.rotation_deg += coarse_rotation
                
                # Scale
                elif key == 'z':
                    node.scale = max(0.1, node.scale - node.scale_step)
                elif key == 'x':
                    node.scale += node.scale_step
                elif key == 'Z':
                    node.scale = max(0.1, node.scale - coarse_scale)
                elif key == 'X':
                    node.scale += coarse_scale

                # Adjust step intensity
                elif key == 't':
                    node.translation_step = max(0.01, node.translation_step - 0.01)
                elif key == 'g':
                    node.translation_step += 0.01
                elif key == 'y':
                    node.rotation_step = max(0.5, node.rotation_step - 0.5)
                elif key == 'k':
                    node.rotation_step += 0.5
                elif key == 'u':
                    node.scale_step = max(0.01, node.scale_step - 0.01)
                elif key == 'j':
                    node.scale_step += 0.01
                
                # Utility
                elif key == 'r':
                    node.reset_params()
                    changed = False
                elif key == 'p':
                    node.print_current_params()
                    changed = False
                elif key == 'h':
                    node.print_instructions()
                    node.print_current_params()
                    changed = False
                elif key == 'c':
                    node.print_launch_params()
                    changed = False
                elif key == '\x1b':  # ESC
                    print("\nExiting...")
                    break
                else:
                    changed = False
                
                if changed:
                    node.print_current_params()
    
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
