"""
Coordinate Frame Test Script for QCar Camera-LiDAR Setup
========================================================

This script helps you verify and adjust the coordinate frame setup
between the camera and LiDAR on the QCar robot.

Usage:
1. Run this script first to test coordinate transformations
2. Place a known object at a specific location in the simulation
3. Use the diagnostic functions to verify alignment
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../libraries/python/hal/utilities'))

import numpy as np
import math
from hal.utilities.geometry import Geometry, MobileRobotGeometry
from pytransform3d import rotations as pr
from pytransform3d import transformations as pt

class CoordinateFrameTester:
    """Test and debug coordinate frame setup"""
    
    def __init__(self):
        self.geometry = MobileRobotGeometry()
        self.setup_test_frames()
    
    def setup_test_frames(self):
        """Setup test coordinate frames"""
        print("=== Setting up test coordinate frames ===")
        
        # Option 1: Corrected basic setup
        camera_pos_1 = [0.15, 0.0, 0.10]
        # Fixed rotation matrix - proper coordinate conversion
        camera_rot_1 = np.array([
            [0, -1, 0],   # Camera X (right) -> Robot -Y (right)
            [0, 0, -1],   # Camera Y (down) -> Robot -Z (down)
            [1, 0, 0]     # Camera Z (forward) -> Robot X (forward)
        ])
        
        # Option 2: QCar-specific setup with proper rotation
        camera_pos_2 = [0.12, 0.0, 0.08]
        camera_rotation_euler = [0, -0.1, 0]  # Slight downward tilt
        # Use the new function name
        camera_rotation = pr.matrix_from_euler(camera_rotation_euler, 'xyz', extrinsic=True)
        
        # Apply coordinate system conversion (corrected)
        coord_conversion = np.array([
            [0, -1, 0],   # Camera X (right) -> Robot -Y (right)  
            [0, 0, -1],   # Camera Y (down) -> Robot -Z (down)
            [1, 0, 0]     # Camera Z (forward) -> Robot X (forward)
        ])
        camera_rot_2 = coord_conversion @ camera_rotation
        
        # Add both camera frames for comparison
        self.geometry.add_frame('camera_v1', p=camera_pos_1, R=camera_rot_1, base='body')
        self.geometry.add_frame('camera_v2', p=camera_pos_2, R=camera_rot_2, base='body')
        
        # Add LiDAR frame
        lidar_pos = [0.0, 0.0, 0.15]
        lidar_rot = np.eye(3)
        self.geometry.add_frame('lidar', p=lidar_pos, R=lidar_rot, base='body')
        
        print("✓ Test frames created: camera_v1, camera_v2, lidar")
    
    def compare_camera_setups(self):
        """Compare different camera setup options"""
        print("\n=== Camera Setup Comparison ===")
        
        # Test point in LiDAR frame (1m forward, 0.5m right)
        test_point_lidar = np.array([0.5, 1.0, 0.0, 1.0])  # [right, forward, up]
        
        print(f"Test point in LiDAR frame: {test_point_lidar[0:3]}")
        
        # Transform to both camera setups
        T_cam1_lidar = self.geometry.get_transform('lidar', 'camera_v1')
        T_cam2_lidar = self.geometry.get_transform('lidar', 'camera_v2') 
        
        point_cam1 = T_cam1_lidar @ test_point_lidar
        point_cam2 = T_cam2_lidar @ test_point_lidar
        
        print(f"Point in camera_v1: {point_cam1[0:3]}")
        print(f"Point in camera_v2: {point_cam2[0:3]}")
        
        # Check if point is in front of cameras
        print(f"Camera_v1 - In front: {point_cam1[2] > 0}, Z-depth: {point_cam1[2]:.3f}")
        print(f"Camera_v2 - In front: {point_cam2[2] > 0}, Z-depth: {point_cam2[2]:.3f}")
    
    def test_lidar_to_camera_projection(self):
        """Test LiDAR point projection to camera"""
        print("\n=== LiDAR to Camera Projection Test ===")
        
        # Simulate some LiDAR points
        angles = np.array([-0.5, -0.25, 0, 0.25, 0.5])  # -30° to +30°
        distances = np.array([2.0, 1.5, 1.0, 1.8, 2.2])
        
        print(f"Test LiDAR points:")
        print(f"  Angles: {np.degrees(angles)}")
        print(f"  Distances: {distances}")
        
        # Convert to LiDAR cartesian
        x_lidar = distances * np.sin(angles)  # right
        y_lidar = distances * np.cos(angles)  # forward
        z_lidar = np.zeros_like(distances)    # up (assume planar scan)
        
        print(f"  LiDAR cartesian: x={x_lidar}, y={y_lidar}")
        
        # Project to camera (v2 setup)
        T_cam_lidar = self.geometry.get_transform('lidar', 'camera_v2')
        
        camera_fov_h = math.radians(70)
        camera_fov_v = math.radians(45)
        image_width, image_height = 640, 480
        
        projected_points = []
        for i in range(len(angles)):
            point_lidar = np.array([x_lidar[i], y_lidar[i], z_lidar[i], 1.0])
            point_camera = T_cam_lidar @ point_lidar
            
            x_cam, y_cam, z_cam = point_camera[0], point_camera[1], point_camera[2]
            
            if z_cam > 0.1:  # In front of camera
                # Project to image
                fx = image_width / (2 * np.tan(camera_fov_h / 2))
                fy = image_height / (2 * np.tan(camera_fov_v / 2))
                cx, cy = image_width / 2, image_height / 2
                
                u = int(fx * x_cam / z_cam + cx)
                v = int(fy * y_cam / z_cam + cy)
                
                projected_points.append((u, v))
                print(f"  Point {i+1}: LiDAR({x_lidar[i]:.2f}, {y_lidar[i]:.2f}) -> Camera({x_cam:.2f}, {y_cam:.2f}, {z_cam:.2f}) -> Pixel({u}, {v})")
            else:
                print(f"  Point {i+1}: Behind camera")
        
        return projected_points
    
    def analyze_angular_mapping(self):
        """Analyze how LiDAR angles map to camera pixels"""
        print("\n=== Angular Mapping Analysis ===")
        
        # Test different LiDAR angles
        test_angles = np.linspace(-math.radians(60), math.radians(60), 13)
        test_distance = 2.0  # Fixed distance
        
        print("LiDAR Angle -> Expected Pixel X")
        print("=" * 35)
        
        T_cam_lidar = self.geometry.get_transform('lidar', 'camera_v2')
        
        camera_fov_h = math.radians(70)
        image_width = 640
        
        for angle in test_angles:
            # LiDAR point
            x_lidar = test_distance * np.sin(angle)
            y_lidar = test_distance * np.cos(angle) 
            point_lidar = np.array([x_lidar, y_lidar, 0.0, 1.0])
            
            # Transform to camera
            point_camera = T_cam_lidar @ point_lidar
            x_cam, y_cam, z_cam = point_camera[0], point_camera[1], point_camera[2]
            
            if z_cam > 0:
                # Project to pixel
                fx = image_width / (2 * np.tan(camera_fov_h / 2))
                u = fx * x_cam / z_cam + image_width / 2
                
                print(f"{math.degrees(angle):6.1f}° -> Pixel {u:6.1f}")
            else:
                print(f"{math.degrees(angle):6.1f}° -> Behind camera")
    
    def get_transformation_matrices(self):
        """Get and display all transformation matrices"""
        print("\n=== Transformation Matrices ===")
        
        frames = ['body', 'camera_v1', 'camera_v2', 'lidar']
        
        for from_frame in frames:
            for to_frame in frames:
                if from_frame != to_frame:
                    try:
                        T = self.geometry.get_transform(from_frame, to_frame)
                        print(f"\nT_{to_frame}_{from_frame}:")
                        print(T)
                    except:
                        print(f"\nT_{to_frame}_{from_frame}: Not available")


def main():
    """Run coordinate frame tests"""
    print("QCar Coordinate Frame Testing Tool")
    print("=" * 40)
    
    tester = CoordinateFrameTester()
    
    # Run all tests
    tester.compare_camera_setups()
    tester.test_lidar_to_camera_projection()
    tester.analyze_angular_mapping()
    
    # Interactive mode
    print("\n" + "=" * 50)
    print("Interactive Testing Mode")
    print("Commands:")
    print("  'matrices' - Show transformation matrices")
    print("  'test <angle> <distance>' - Test specific LiDAR point")
    print("  'exit' - Exit")
    
    while True:
        try:
            cmd = input("\n> ").strip().lower()
            
            if cmd == 'exit':
                break
            elif cmd == 'matrices':
                tester.get_transformation_matrices()
            elif cmd.startswith('test'):
                parts = cmd.split()
                if len(parts) == 3:
                    angle_deg = float(parts[1])
                    distance = float(parts[2])
                    
                    # Test specific point
                    angle_rad = math.radians(angle_deg)
                    x_lidar = distance * np.sin(angle_rad)
                    y_lidar = distance * np.cos(angle_rad)
                    
                    T_cam_lidar = tester.geometry.get_transform('lidar', 'camera_v2')
                    point_lidar = np.array([x_lidar, y_lidar, 0.0, 1.0])
                    point_camera = T_cam_lidar @ point_lidar
                    
                    print(f"LiDAR point: angle={angle_deg}°, distance={distance}m")
                    print(f"Cartesian: ({x_lidar:.2f}, {y_lidar:.2f}, 0.0)")
                    print(f"Camera frame: ({point_camera[0]:.2f}, {point_camera[1]:.2f}, {point_camera[2]:.2f})")
                else:
                    print("Usage: test <angle_degrees> <distance_meters>")
            else:
                print("Unknown command. Type 'exit' to quit.")
                
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Error: {e}")
    
    print("Testing complete!")


if __name__ == "__main__":
    main()
