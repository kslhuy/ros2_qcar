"""
QCar 2 Library Example - Semantic Occupancy Grid Mapping with YOLO + RealSense RGBD + LiDAR
--------------------------------------------------------------------------------------------

This example demonstrates how to create a semantic occupancy grid map using:
- YOLO for object detection and semantic segmentation
- RealSense RGBD camera for depth information
- LiDAR for geometric occupancy mapping
- Fusion of all sensor data into a unified semantic occupancy grid

The system creates a real-time semantic occupancy grid map that shows both
geometric occupancy (from LiDAR) and semantic labels (from YOLO detections).

.. note::
    Make sure you have Quanser Interactive Labs open before running this
    example. This example is designed to best be run in QCar Cityscape.
"""

import sys
import os
import argparse
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../libraries/python/hal/utilities'))

from qvl.qlabs import QuanserInteractiveLabs
from qvl.free_camera import QLabsFreeCamera
from qvl.qcar2 import QLabsQCar2
import time
import math
import numpy as np
import cv2
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets
from qvl.system import QLabsSystem
from collections import defaultdict
import traceback
# Removed matplotlib imports for camera-only version
from scipy.spatial.distance import cdist

# Import utility classes
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../libraries/python/hal/utilities'))

from  hal.utilities.mapping import OccupancyGrid, OGRPLidarModel, RollingOccupancyGrid
from hal.utilities.image_processing import ImageProcessing
from hal.utilities.geometry import Geometry, MobileRobotGeometry

# Import pytransform3d for coordinate transformations
from pytransform3d import rotations as pr
from pytransform3d import transformations as pt

# YOLO import
# from pit.YOLO.nets import YOLOv8

class LiDARCameraCalibration:
    """
    Enhanced LiDAR-Camera calibration based on Early_Fusion techniques.
    Handles coordinate transformations between 2D LiDAR and camera frames.
    """
    def __init__(self, geometry_manager):
        self.geometry_manager = geometry_manager
        
        # Camera intrinsic parameters (from QCar setup)
        self.fx = 640.0  # Focal length x 
        self.fy = 480.0  # Focal length y
        self.cx = 320.0  # Principal point x
        self.cy = 240.0  # Principal point y
        
        # Camera intrinsic matrix
        self.K = np.array([[self.fx, 0, self.cx],
                          [0, self.fy, self.cy],
                          [0, 0, 1]], dtype=np.float32)
        
        # LiDAR to Camera transformation (2D->3D projection)
        # Since QCar LiDAR is 2D, we need to project to camera's 3D view
        self.lidar_to_cam_translation = np.array([0.0, 0.0, 0.3])  # LiDAR height offset
        self.lidar_to_cam_rotation = np.eye(3)  # Assume aligned for now
        
    def project_lidar_to_image_plane(self, lidar_angles, lidar_distances):
        """
        Project 2D LiDAR points to image plane using camera model
        Similar to Early_Fusion's project_velo_to_image but adapted for 2D LiDAR
        """
        # Convert 2D LiDAR to 3D points (assume z=0 for 2D scan plane)
        x_coords = lidar_distances * np.cos(lidar_angles)
        y_coords = lidar_distances * np.sin(lidar_angles)
        z_coords = np.zeros_like(x_coords)
        
        # Create homogeneous coordinates
        lidar_points_3d = np.column_stack([x_coords, y_coords, z_coords, np.ones(len(x_coords))])
        
        # Transform to camera coordinates
        # Apply translation offset (LiDAR to camera)
        cam_points = lidar_points_3d[:, :3] + self.lidar_to_cam_translation
        
        # Filter points in front of camera
        front_mask = cam_points[:, 2] > 0.1  # At least 10cm in front
        cam_points = cam_points[front_mask]
        original_indices = np.where(front_mask)[0]
        
        if len(cam_points) == 0:
            return np.array([]), np.array([])
        
        # Project to image plane using camera intrinsics
        image_points = self.K @ cam_points.T
        
        # Convert from homogeneous to Cartesian
        image_points[0, :] /= image_points[2, :]
        image_points[1, :] /= image_points[2, :]
        
        # Return 2D image coordinates
        projected_points = image_points[:2, :].T
        
        return projected_points, original_indices
    
    def get_lidar_in_image_fov(self, lidar_angles, lidar_distances, image_width, image_height, clip_distance=0.5):
        """
        Filter LiDAR points to only those visible in camera FOV
        Based on Early_Fusion's get_lidar_in_image_fov
        """
        projected_points, valid_indices = self.project_lidar_to_image_plane(lidar_angles, lidar_distances)
        
        if len(projected_points) == 0:
            return np.array([]), np.array([]), np.array([])
        
        # Filter points within image bounds
        x_valid = (projected_points[:, 0] >= 0) & (projected_points[:, 0] < image_width)
        y_valid = (projected_points[:, 1] >= 0) & (projected_points[:, 1] < image_height)
        distance_valid = lidar_distances[valid_indices] > clip_distance
        
        fov_mask = x_valid & y_valid & distance_valid
        
        fov_points = projected_points[fov_mask]
        fov_distances = lidar_distances[valid_indices][fov_mask]
        fov_angles = lidar_angles[valid_indices][fov_mask]
        
        return fov_points, fov_distances, fov_angles
from YOLOv8Wrapper_Huy import YOLOv8Wrapper_Huy



class RobotGeometryManager:
    """
    Manages coordinate frame transformations for the QCar robot using the geometry class
    """
    def __init__(self):
        # Initialize geometry manager
        self.geometry = MobileRobotGeometry()
        
        is_physical = False
        self.scale = 0.1 if is_physical else 1.0  # Scale for physical QCar
        # Define coordinate frames
        self.setup_coordinate_frames()
        # Camera parameters
        self.camera_fov_h = math.radians(70)  # Horizontal FOV
        self.camera_fov_v = math.radians(45)  # Vertical FOV
        
        print("Robot Geometry Manager initialized with coordinate frames")
    
    def setup_coordinate_frames(self):
        """Set up all coordinate frames relative to the robot body
        IMPORTANCE : 
        # Your positions are in full-scale units, which is correct for the virtual environment. 
        # If you're working with a physical QCar, you'd need to scale positions by 0.1 (e.g., camera at [0.095, 0.032, 0.172] m).

        """

        
        # Add camera frame (FIXED coordinate transformation for proper LiDAR-camera alignment)
        # Camera frame: X=right, Y=down, Z=forward (OpenCV convention)
        # Robot frame: X=forward, Y=left, Z=up (ROS REP-103)
        camera_position = [0.95*self.scale, 0.32*self.scale, 1.72*self.scale]
        # CORRECTED Camera-to-Robot transformation:
        # Camera X (right) → Robot -Y (right in robot frame)
        # Camera Y (down) → Robot -Z (down in robot frame) 
        # Camera Z (forward) → Robot X (forward in robot frame)
        camera_rotation = np.array([
            [0, 0, 1],    # Camera Z → Robot X (forward)
            [-1, 0, 0],   # Camera X → Robot -Y (right)
            [0, -1, 0]    # Camera Y → Robot -Z (down)
        ])
        self.geometry.add_frame(
            name='camera', 
            p=camera_position, 
            R=camera_rotation, 
            base='body'
        )
        
        # Add LiDAR frame (assume LiDAR is mounted at the center, same orientation as body)
        lidar_position = [-0.12*self.scale, 0.0, 1.93*self.scale]
        lidar_rotation = np.eye(3)  # Same orientation as body
        self.geometry.add_frame(
            name='lidar', 
            p=lidar_position, 
            R=lidar_rotation, 
            base='body'
        )
        
        print("Coordinate frames set up:")
        print(f"  - world (default frame)")
        print(f"  - body (robot center)")
        print(f"  - camera (forward-looking camera)")
        print(f"  - lidar (top-mounted LiDAR)")
        
        # Verify coordinate frame setup
        self.verify_coordinate_frames()
    
    def verify_coordinate_frames(self):
        """Verify that coordinate frames are set up correctly"""
        print("\n=== Coordinate Frame Verification ===")
        
        # Check camera frame transformation
        T_camera_body = self.geometry.get_transform('camera', 'body')
        T_lidar_body = self.geometry.get_transform('lidar', 'body')
        T_camera_lidar = self.geometry.get_transform('lidar', 'camera')
        
        print("Camera frame relative to body:")
        print(f"  Position: {self.geometry.get_translation('camera', 'body')}")
        print(f"  Rotation matrix:\n{self.geometry.get_rotation_rm('camera', 'body')}")
        
        print("LiDAR frame relative to body:")
        print(f"  Position: {self.geometry.get_translation('lidar', 'body')}")
        print(f"  Rotation matrix:\n{self.geometry.get_rotation_rm('lidar', 'body')}")
        
        print("LiDAR to Camera transformation:")
        print(f"  Translation: {T_camera_lidar[0:3, 3]}")
        
        # Test point transformation - CORRECTED test points
        test_point_lidar = np.array([1.0, 0.0, 0.0, 1.0])  # 1m forward in LiDAR frame
        test_point_camera = T_camera_lidar @ test_point_lidar
        print(f"Test point [1,0,0] (forward) in LiDAR -> {test_point_camera[0:3]} in Camera")
        
        test_point_lidar = np.array([0.0, 1.0, 0.0, 1.0])  # 1m left in LiDAR frame
        test_point_camera = T_camera_lidar @ test_point_lidar
        print(f"Test point [0,1,0] (left) in LiDAR -> {test_point_camera[0:3]} in Camera")
        
        test_point_lidar = np.array([0.0, -1.0, 0.0, 1.0])  # 1m right in LiDAR frame
        test_point_camera = T_camera_lidar @ test_point_lidar
        print(f"Test point [0,-1,0] (right) in LiDAR -> {test_point_camera[0:3]} in Camera")
        print("NOTE: In camera frame: +X=right, +Y=down, +Z=forward")
        
    def setup_coordinate_frames_qcar_specific(self):
        """Set up coordinate frames based on actual QCar specifications with FIXED transformations"""
        
        # QCar Camera mounting (more accurate based on actual robot)
        # Camera is typically mounted forward-facing, slightly angled down
        camera_position = [0.95*self.scale, 0.32*self.scale, 1.72*self.scale]
        
        # CORRECTED Camera coordinate system transformation
        # Camera coordinate system: X=right, Y=down, Z=forward (OpenCV/ROS convention)
        # Robot coordinate system: X=forward, Y=left, Z=up (ROS REP-103)
        # camera_rotation_euler = [0, -0.1, 0]  # Slight downward tilt (10 degrees)
        # camera_rotation = pr.matrix_from_euler(camera_rotation_euler, 'xyz', extrinsic=True)
        
        # Apply coordinate system conversion (CORRECTED for proper LiDAR-camera alignment)
        coord_conversion = np.array([
            [0, 0, 1],    # Camera Z (forward) -> Robot X (forward)
            [-1, 0, 0],   # Camera X (right) -> Robot -Y (right)
            [0, -1, 0]    # Camera Y (down) -> Robot -Z (down)
        ])

        camera_rotation = np.array([
            [0, 0, 1],    # Camera Z → Robot X (forward)
            [-1, 0, 0],   # Camera X → Robot -Y (right)
            [0, -1, 0]    # Camera Y → Robot -Z (down)
        ])
        
        # final_camera_rotation = coord_conversion @ camera_rotation
        
        self.geometry.add_frame(
            name='camera', 
            p=camera_position, 
            R=camera_rotation, 
            base='body'
        )
        
        # QCar LiDAR mounting (typically centered and level)
        # Add LiDAR frame (assume LiDAR is mounted at the center, same orientation as body)
        lidar_position = [-0.12*self.scale, 0.0, 1.93*self.scale]
        lidar_rotation = np.eye(3)  # Same orientation as body
        self.geometry.add_frame(
            name='lidar', 
            p=lidar_position, 
            R=lidar_rotation, 
            base='body'
        )
    
    def calibrate_camera_lidar_alignment(self, detection_objects, lidar_angles, lidar_distances , verbose=False):
        """
        Calibrate camera-LiDAR alignment by analyzing detected objects
        
        Args:
            detection_objects: List of enhanced detection objects with center coordinates
            lidar_angles: LiDAR scan angles
            lidar_distances: LiDAR scan distances
        """
        print("\n=== Camera-LiDAR Calibration ===")
        
        if not detection_objects:
            print("No detections available for calibration")
            return 0.0
        
        # For each detection, find the closest LiDAR point and calculate offset
        angle_offsets = []
        
        for detection in detection_objects:
            if not hasattr(detection, 'center_x'):
                continue
                
            # Calculate expected angle from camera pixel
            # FIXED: Invert angle direction for consistency with LiDAR coordinate system
            image_width = 640  # Assume standard resolution
            pixel_angle = (detection.center_x / image_width - 0.5) * self.camera_fov_h
            
            # Find closest LiDAR points
            angle_diffs = np.abs(lidar_angles - pixel_angle)
            closest_idx = np.argmin(angle_diffs)
            
            if lidar_distances[closest_idx] > 0.5:  # Valid distance
                lidar_angle = lidar_angles[closest_idx]
                angle_offset = lidar_angle - pixel_angle
                angle_offsets.append(angle_offset)

                if verbose:
                    print(f"Detection at pixel {detection.center_x:.0f}")
                    print(f"  Expected angle: {math.degrees(pixel_angle):.1f}°")
                    print(f"  LiDAR angle: {math.degrees(lidar_angle):.1f}°")
                    print(f"  Offset: {math.degrees(angle_offset):.1f}°")

        if angle_offsets:
            mean_offset = np.mean(angle_offsets)
            if verbose:
                print(f"\nMean angular offset: {math.degrees(mean_offset):.1f}°")
            if abs(mean_offset) > math.radians(5):  # More than 5 degrees offset
                print(f"Large offset detected! Consider adjusting camera rotation by {math.degrees(mean_offset):.1f}°")
                return mean_offset
            else:
                print("Camera-LiDAR alignment looks good!")
        
        return 0.0
    
    def adjust_camera_rotation(self, yaw_adjustment_rad):
        """Adjust camera rotation based on calibration results"""
        print(f"Adjusting camera yaw by {math.degrees(yaw_adjustment_rad):.1f}°")
        
        # Get current camera rotation
        current_rotation = self.geometry.get_rotation_rm('camera', 'body')
        
        # Create yaw adjustment matrix
        yaw_adjustment = pr.active_matrix_from_extrinsic_euler_xyz([0, 0, yaw_adjustment_rad])
        
        # Apply adjustment
        new_rotation = current_rotation @ yaw_adjustment
        self.geometry.set_rotation_rm(new_rotation, 'camera', 'body')
        
        print("Camera rotation adjusted!")
        self.verify_coordinate_frames()
    
    def lidar_to_camera_projection(self, angles, distances, image_width=640, image_height=480):
        """
        Project LiDAR points to camera image coordinates
        
        Args:
            angles: LiDAR scan angles (radians)
            distances: LiDAR scan distances (meters)  
            image_width: Camera image width
            image_height: Camera image height
            
        Returns:
            projected_points: List of (u, v, depth) tuples for valid projections
            lidar_points_3d: 3D points in camera frame
        """
        projected_points = []
        lidar_points_3d = []
        
        # Convert LiDAR scan to 3D points in LiDAR frame
        # FIXED: QCar LiDAR convention - angle 0 is forward, positive angles go counter-clockwise
        # Standard LiDAR coordinate system: X=forward, Y=left, Z=up
        lidar_x = distances * np.cos(angles)  # X=forward (distance * cos(angle))
        lidar_y = - distances * np.sin(angles)  # Y=left (distance * sin(angle))
        lidar_z = np.zeros_like(distances)    # Z=up (planar scan at height=0)
        
        # Transform LiDAR points to camera frame
        for i in range(len(angles)):
            if distances[i] > 0.1 and distances[i] < 50.0:  # Valid range
                # Point in LiDAR frame
                point_lidar = np.array([lidar_x[i], lidar_y[i], lidar_z[i], 1.0])
                
                # Transform to camera frame
                T_camera_lidar = self.geometry.get_transform('lidar', 'camera')
                point_camera = T_camera_lidar @ point_lidar
                
                # Extract 3D coordinates in camera frame
                x_cam, y_cam, z_cam = point_camera[0], point_camera[1], point_camera[2]
                
                # Check if point is in front of camera
                if z_cam > 0.1:  # At least 10cm in front
                    # Project to image coordinates using pinhole camera model
                    # Assuming camera intrinsic matrix
                    fx = image_width / (2 * np.tan(self.camera_fov_h / 2))
                    fy = image_height / (2 * np.tan(self.camera_fov_v / 2))
                    cx = image_width / 2
                    cy = image_height / 2
                    
                    u = int(fx * x_cam / z_cam + cx)
                    v = int(fy * y_cam / z_cam + cy)
                    
                    # Check if projection is within image bounds
                    if 0 <= u < image_width and 0 <= v < image_height:
                        projected_points.append((u, v, z_cam))
                        lidar_points_3d.append((x_cam, y_cam, z_cam))
        
        return projected_points, lidar_points_3d
    
    def project_semantic_objects_to_lidar(self, detection_objects, image_width=640, image_height=480):
        """
        Project semantic objects from camera to LiDAR coordinate frame
        
        Args:
            detection_objects: List of enhanced detection objects
            
        Returns:
            projected_objects: List of objects with corrected 3D positions
        """
        projected_objects = []
        
        for detection in detection_objects:
            if hasattr(detection, 'distance') and detection.distance > 0:
                # Use object center coordinates
                center_u = detection.center_x
                center_v = detection.center_y
                distance = detection.distance
                
                # Convert to camera frame 3D point
                fx = image_width / (2 * np.tan(self.camera_fov_h / 2))
                fy = image_height / (2 * np.tan(self.camera_fov_v / 2))
                cx = image_width / 2
                cy = image_height / 2
                
                # Assuming the distance is the Z coordinate in camera frame
                z_cam = distance
                x_cam = (center_u - cx) * z_cam / fx
                y_cam = (center_v - cy) * z_cam / fy
                
                # Transform to LiDAR frame for correct positioning
                point_camera = np.array([x_cam, y_cam, z_cam, 1.0])
                T_lidar_camera = self.geometry.get_transform('camera', 'lidar')
                point_lidar = T_lidar_camera @ point_camera
                
                # Create corrected detection object
                corrected_detection = {
                    'bbox': detection.bbox,
                    'center_x': detection.center_x,
                    'center_y': detection.center_y,
                    'width': detection.width,
                    'height': detection.height,
                    'area': detection.area,
                    'confidence': detection.conf,
                    'class_id': detection.class_id,
                    'class_name': detection.name,
                    'distance': detection.distance,
                    'lidar_x': point_lidar[0],
                    'lidar_y': point_lidar[1],
                    'lidar_z': point_lidar[2]
                }
                
                # Calculate corrected angle and distance in LiDAR frame
                # FIXED: Correct angle calculation for LiDAR coordinate system
                # In LiDAR frame: X=forward, Y=left
                # Angle should be arctan2(Y, X) for proper direction
                corrected_angle = np.arctan2(point_lidar[1], point_lidar[0])  # arctan2(Y, X)
                corrected_distance = np.sqrt(point_lidar[0]**2 + point_lidar[1]**2)
                
                corrected_detection['corrected_angle'] = math.degrees(corrected_angle)
                corrected_detection['corrected_distance'] = corrected_distance
                
                # Handle traffic light color if available
                if hasattr(detection, 'lightColor'):
                    corrected_detection['traffic_light_color'] = detection.lightColor
                
                projected_objects.append(corrected_detection)
        
        return projected_objects
    
    def update_robot_pose(self, x, y, theta):
        """Update robot pose in world frame"""
        self.geometry.set_pose_2d([x, y, theta], frame='body', base='world')


class SemanticOccupancyGrid:
    """
    Enhanced Occupancy Grid with semantic information from YOLO detections
    """
    def __init__(self, xLength=20, yLength=20, cellWidth=0.2, pPrior=0.5):
        # Initialize base occupancy grid
        self.base_ogm = OccupancyGrid(
            xLength=xLength, 
            yLength=yLength, 
            cellWidth=cellWidth,
            sensor=OGRPLidarModel(cellWidth, filter=True),
            pPrior=pPrior
        )
        
        # Semantic layer - stores class labels for each cell
        self.semantic_map = np.zeros((self.base_ogm.m, self.base_ogm.n), dtype=np.int32)
        self.confidence_map = np.zeros((self.base_ogm.m, self.base_ogm.n), dtype=np.float32)
        
        # Class mapping for YOLO COCO dataset
        self.class_names = {
            0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 9: 'traffic light',
            11: 'stop sign', 12: 'parking meter',
        }
        
        # Colors for visualization (RGB)
        self.class_colors = {
            0: [255, 0, 0],    # person - red
            2: [0, 255, 0],    # car - green  
            5: [0, 0, 255],    # bus - blue
            7: [255, 255, 0],  # truck - yellow
            9: [255, 0, 255],  # traffic light - magenta
            11: [0, 255, 255], # stop sign - cyan
            -1: [128, 128, 128] # unknown - gray
        }
        
        # Image processing utilities
        self.img_proc = ImageProcessing()
        
        print(f"Initialized Semantic OGM: {self.base_ogm.m}x{self.base_ogm.n} cells, "
              f"cell size: {cellWidth}m, area: {xLength}x{yLength}m")
    
    def update_with_lidar(self, x, y, theta, angles, distances):
        """Update occupancy grid with LiDAR data"""
        self.base_ogm.update(x, y, theta, angles, distances)
    
    def update_with_semantic_detection(self, detection, depth_image=None, camera_intrinsics=None):
        """
        Update semantic map with YOLO detection (using dict format)
        
        Args:
            detection: Dict with detection information
            depth_image: Depth image from RealSense (optional)
            camera_intrinsics: Camera intrinsic parameters (optional)
        """
        # Use corrected measurements if available, otherwise fall back to original
        if 'corrected_distance' in detection and 'corrected_angle' in detection:
            distance = detection['corrected_distance']
            angle = math.radians(detection['corrected_angle'])
        elif 'distance' in detection and 'angle' in detection:
            distance = detection['distance']
            angle = math.radians(detection['angle'])
        else:
            return
        
        # Object position relative to robot
        obj_x = distance * math.cos(angle)
        obj_y = distance * math.sin(angle)
        
        # Convert to grid coordinates
        i, j = self.base_ogm.xy_to_ij_rounded(obj_x, obj_y)
        
        # Check if coordinates are within grid bounds
        if 0 <= i < self.base_ogm.m and 0 <= j < self.base_ogm.n:
            class_id = detection.get('class_id', -1)
            confidence = detection.get('confidence', 0.0)
            
            # Update semantic map if confidence is higher than existing
            if confidence > self.confidence_map[i, j]:
                self.semantic_map[i, j] = class_id
                self.confidence_map[i, j] = confidence
                
            # Also mark surrounding cells (object extent)
            radius_cells = max(1, int(1.0 / self.base_ogm.cellWidth))  # 1 meter radius
            for di in range(-radius_cells, radius_cells + 1):
                for dj in range(-radius_cells, radius_cells + 1):
                    ni, nj = i + di, j + dj
                    if (0 <= ni < self.base_ogm.m and 0 <= nj < self.base_ogm.n and
                        di*di + dj*dj <= radius_cells*radius_cells):
                        
                        # Reduce confidence for surrounding cells
                        surrounding_confidence = confidence * 0.5
                        if surrounding_confidence > self.confidence_map[ni, nj]:
                            self.semantic_map[ni, nj] = class_id
                            self.confidence_map[ni, nj] = surrounding_confidence
    
    def update_with_depth(self, depth_image, rgb_image, camera_intrinsics, pose):
        """
        Update occupancy grid using depth image
        
        Args:
            depth_image: Depth image from RealSense
            rgb_image: RGB image for correspondence
            camera_intrinsics: Camera intrinsic matrix
            pose: Robot pose [x, y, theta]
        """
        if depth_image is None or camera_intrinsics is None:
            return
            
        # Handle both grayscale depth images (2D) and color depth images (3D)
        if len(depth_image.shape) == 3:
            print(f"Debug: Depth image shape before conversion: {depth_image.shape}")
            # Color depth image - convert to grayscale or extract depth channel
            if depth_image.shape[2] == 3:  # RGB image
                # Convert to grayscale and use as depth approximation
                depth_image = cv2.cvtColor(depth_image, cv2.COLOR_BGR2GRAY)
            else:
                # Take first channel as depth
                depth_image = depth_image[:, :, 0]
            print(f"Debug: Depth image shape after conversion: {depth_image.shape}")
        
        height, width = depth_image.shape
        fx, fy = camera_intrinsics[0, 0], camera_intrinsics[1, 1]
        cx, cy = camera_intrinsics[0, 2], camera_intrinsics[1, 2]
        
        # Sample depth image at regular intervals to avoid processing every pixel
        step = 10  # Process every 10th pixel
        
        for v in range(0, height, step):
            for u in range(0, width, step):
                depth = depth_image[v, u] / 1000.0  # Convert mm to meters
                
                if 0.5 < depth < 10.0:  # Valid depth range
                    # Convert pixel to 3D point in camera frame
                    x_cam = (u - cx) * depth / fx
                    y_cam = (v - cy) * depth / fy
                    z_cam = depth
                    
                    # Transform to robot frame (assuming camera is forward-facing)
                    x_robot = z_cam  # Camera Z is robot X (forward)
                    y_robot = -x_cam  # Camera X is robot -Y (left)
                    
                    # Transform to world frame
                    cos_theta, sin_theta = math.cos(pose[2]), math.sin(pose[2])
                    x_world = pose[0] + x_robot * cos_theta - y_robot * sin_theta
                    y_world = pose[1] + x_robot * sin_theta + y_robot * cos_theta
                    
                    # Update occupancy grid
                    i, j = self.base_ogm.xy_to_ij_rounded(x_world, y_world)
                    if 0 <= i < self.base_ogm.m and 0 <= j < self.base_ogm.n:
                        # Mark as occupied with high confidence
                        self.base_ogm.lMap[i, j] = min(self.base_ogm.lMap[i, j] + 2.0, 
                                                      self.base_ogm._lMax)
        
        self.base_ogm._pMapOutOfDate = True
    
    def get_semantic_visualization(self):
        """
        Create a colored visualization of the semantic occupancy grid
        
        Returns:
            RGB image of the semantic map
        """
        # Create base occupancy visualization
        occupancy = self.base_ogm.pMap
        
        # Create RGB image
        height, width = occupancy.shape
        semantic_vis = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Color based on occupancy probability
        for i in range(height):
            for j in range(width):
                prob = occupancy[i, j]
                semantic_class = self.semantic_map[i, j]
                confidence = self.confidence_map[i, j]
                
                if prob > 0.7:  # High occupancy
                    if semantic_class in self.class_colors and confidence > 0.3:
                        # Use semantic class color
                        color = self.class_colors[semantic_class]
                        semantic_vis[i, j] = [int(c * confidence) for c in color]
                    else:
                        # Unknown object - white
                        intensity = int(255 * prob)
                        semantic_vis[i, j] = [intensity, intensity, intensity]
                elif prob > 0.3:  # Medium occupancy - gray
                    intensity = int(128 * prob)
                    semantic_vis[i, j] = [intensity, intensity, intensity]
                # else: free space - black (already initialized)
        
        return semantic_vis
    
    def get_occupancy_map(self):
        """Get the base occupancy probability map"""
        return self.base_ogm.pMap
    
    def get_semantic_map(self):
        """Get the semantic class map"""
        return self.semantic_map
    
    def print_statistics(self):
        """Print statistics about the semantic occupancy grid"""
        occupancy = self.base_ogm.pMap
        occupied_cells = np.sum(occupancy > 0.7)
        free_cells = np.sum(occupancy < 0.3)
        unknown_cells = np.sum((occupancy >= 0.3) & (occupancy <= 0.7))
        
        semantic_cells = np.sum(self.confidence_map > 0.3)
        unique_classes = np.unique(self.semantic_map[self.confidence_map > 0.3])
        
        print(f"\n=== Semantic OGM Statistics ===")
        print(f"Grid size: {self.base_ogm.m} x {self.base_ogm.n} cells")
        print(f"Occupied cells: {occupied_cells}")
        print(f"Free cells: {free_cells}")
        print(f"Unknown cells: {unknown_cells}")
        print(f"Semantic cells: {semantic_cells}")
        print(f"Detected classes: {[self.class_names.get(c, f'class_{c}') for c in unique_classes if c != 0]}")


def draw_lidar_on_camera_image(image, projected_points, lidar_points_3d, detection_objects=None, proximity_threshold=50):
    """
    Draw only LiDAR points that are close to detected object bounding boxes
    
    Args:
        image: Camera image (numpy array)
        projected_points: List of (u, v, depth) tuples
        lidar_points_3d: 3D points in camera frame
        detection_objects: List of detection objects with bounding boxes (optional)
        proximity_threshold: Distance threshold in pixels for LiDAR points to be considered near objects
    
    Returns:
        Annotated image with filtered LiDAR points near detected objects
    """
    annotated_image = image.copy()
    image_height, image_width = image.shape[:2]
    
    # If no detection objects provided, draw all LiDAR points (fallback to original behavior)
    if detection_objects is None or len(detection_objects) == 0:
        for (u, v, depth), (x, y, z) in zip(projected_points, lidar_points_3d):
            # Color based on distance (closer = red, farther = blue)
            max_distance = 20.0
            distance_ratio = min(depth / max_distance, 1.0)
            
            # Create color gradient: close=red(255,0,0), far=blue(0,0,255)
            color_r = int(255 * (1 - distance_ratio))
            color_b = int(255 * distance_ratio)
            color = (color_b, 0, color_r)  # BGR format for OpenCV
            
            # Draw point
            cv2.circle(annotated_image, (u, v), 3, color, -1)
        return annotated_image
    
    # Filter LiDAR points based on proximity to detected objects
    for (u, v, depth), (x, y, z) in zip(projected_points, lidar_points_3d):
        # Check if this LiDAR point is near any detected object
        point_near_object = False
        
        for detection in detection_objects:
            # Check if point is within the bounding box (with some expansion)
            if rect_contains_point(detection, (u, v), image_width, image_height, shrink_factor=-0.2):  # Negative shrink = expand
                point_near_object = True
                break
            
            # Alternative: Check distance to bbox center for proximity
            if hasattr(detection, 'center_x') and hasattr(detection, 'center_y'):
                center_x = detection.center_x
                center_y = detection.center_y
            elif 'center_x' in detection and 'center_y' in detection:
                center_x = detection['center_x']
                center_y = detection['center_y']
            else:
                # Try to get center from bbox if available
                if hasattr(detection, 'bbox') and len(detection.bbox) >= 4:
                    bbox = detection.bbox
                    center_x = (bbox[0] + bbox[2]) / 2
                    center_y = (bbox[1] + bbox[3]) / 2
                elif 'bbox' in detection and len(detection['bbox']) >= 4:
                    bbox = detection['bbox']
                    center_x = (bbox[0] + bbox[2]) / 2
                    center_y = (bbox[1] + bbox[3]) / 2
                else:
                    continue
            
            # Calculate distance from LiDAR point to object center
            distance_to_center = ((u - center_x)**2 + (v - center_y)**2)**0.5
            
            if distance_to_center <= proximity_threshold:
                point_near_object = True
                break
        
        # Only draw LiDAR points that are near detected objects
        if point_near_object:
            # Color based on distance (closer = red, farther = blue)
            max_distance = 20.0
            distance_ratio = min(depth / max_distance, 1.0)
            
            # Create color gradient: close=red(255,0,0), far=blue(0,0,255)
            color_r = int(255 * (1 - distance_ratio))
            color_b = int(255 * distance_ratio)
            color = (color_b, 0, color_r)  # BGR format for OpenCV
            
            # Draw point with slightly larger size to make them more visible
            cv2.circle(annotated_image, (u, v), 4, color, -1)
            # Add small white border for better visibility
            cv2.circle(annotated_image, (u, v), 4, (255, 255, 255), 1)
    
    return annotated_image


def build_probability_colormap(prob_map):
    """Convert probability map (0..1) to a colored image for display."""
    prob_norm = (np.clip(prob_map, 0.0, 1.0) * 255).astype(np.uint8)
    colored = cv2.applyColorMap(prob_norm, cv2.COLORMAP_INFERNO)
    return colored


def build_semantic_overlay(semantic_ogm):
    """Return semantic visualization scaled for easier viewing."""
    sem_vis = semantic_ogm.get_semantic_visualization()
    # Upscale for display if small
    scale = 4 if sem_vis.shape[0] < 400 else 1
    if scale != 1:
        sem_vis = cv2.resize(sem_vis, (sem_vis.shape[1]*scale, sem_vis.shape[0]*scale), interpolation=cv2.INTER_NEAREST)
    return sem_vis


class RealSenseWrapper:
    """
    Wrapper for RealSense camera (simulated with QCar cameras)
    """
    def __init__(self, qcar_handle):
        self.qcar = qcar_handle
        
        # Possessing the RealSense depth camera on the QCar
        self.qcar.possess(self.qcar.CAMERA_DEPTH)
        
        # Simulated camera intrinsics for QCar camera
        # These would typically come from camera calibration
        self.intrinsics = np.array([
            [640, 0, 320],
            [0, 640, 240], 
            [0, 0, 1]
        ], dtype=np.float32)
        
        self.img_proc = ImageProcessing()
    
    def get_frames(self):
        """
        Get RGB and depth frames
        
        Returns:
            rgb_image: RGB image (numpy array)
            depth_image: Depth image (numpy array) - from depth camera, properly scaled
        """
        # Get RGB image
        success_rgb, rgb_image = self.qcar.get_image(camera=self.qcar.CAMERA_RGB)
        crop_height = 40
        # Crop bottom pixels for better performance
        if success_rgb:
            height = rgb_image.shape[0]
            rgb_image = rgb_image[0:height-crop_height, :]  # Remove bottom 40 pixels and use cropped image

        # Get depth image from depth camera
        success_depth, depth_image = self.qcar.get_image(camera=self.qcar.CAMERA_DEPTH)
        
        if success_rgb and success_depth:
            # Crop depth image to match RGB dimensions
            if success_depth:
                depth_height = depth_image.shape[0]
                depth_image = depth_image[0:depth_height-crop_height, :]  # Crop depth image too

            # Process depth image to ensure it's in the right format
            # print(f"Debug: Raw depth image shape: {depth_image.shape}, dtype: {depth_image.dtype}")
            
            if len(depth_image.shape) == 3:
                # QCar depth camera returns RGB-like format, need to extract depth information
                # Method 1: Use intensity as depth proxy (convert to grayscale first)
                depth_gray = cv2.cvtColor(depth_image, cv2.COLOR_BGR2GRAY)
                
                # Method 2: Better depth extraction - use dominant channel or custom conversion
                # For QCar simulation, the depth might be encoded in specific channels
                # Try different approaches based on the actual data
                
                # Approach A: Use blue channel (often used for depth in some systems)
                depth_blue = depth_image[:, :, 0]  # Blue channel
                
                # Approach B: Use weighted combination for better depth approximation
                # depth_weighted = 0.299 * depth_image[:, :, 2] + 0.587 * depth_image[:, :, 1] + 0.114 * depth_image[:, :, 0]
                
                # For now, use grayscale conversion with proper scaling
                depth_image = depth_gray
                # print(f"Debug: Converted depth image shape: {depth_image.shape}")
            
            # Resize depth image to match RGB image dimensions (480, 640)
            target_height, target_width = rgb_image.shape[:2]
            if depth_image.shape != (target_height, target_width):
                # print(f"Debug: Resizing depth from {depth_image.shape} to ({target_height}, {target_width})")
                depth_image = cv2.resize(depth_image, (target_width, target_height), interpolation=cv2.INTER_NEAREST)
                # print(f"Debug: Resized depth image shape: {depth_image.shape}")
            
            # Enhanced depth processing for QCar
            # QCar depth values need proper scaling - adjust based on actual range
            if depth_image.dtype == np.uint8:
                # Scale 8-bit values to realistic depth range (0-50m)
                # Darker pixels = farther, lighter pixels = closer (inverse of typical depth)
                depth_image = (255 - depth_image.astype(np.float32)) / 255.0 * 50.0  # 0-50 meters
                depth_image = (depth_image * 1000).astype(np.uint16)  # Convert to mm as uint16
            else:
                # Convert to uint16 format for depth processing
                depth_image = depth_image.astype(np.uint16)
                
            return rgb_image, depth_image
        elif success_rgb:
            # Fallback: Get CSI_FRONT image and simulate depth
            success_csi, csi_image = self.qcar.get_image(camera=self.qcar.CAMERA_CSI_FRONT)
            if success_csi:
                # Convert CSI to simulated depth with improved scaling
                gray_csi = cv2.cvtColor(csi_image, cv2.COLOR_BGR2GRAY)
                
                # Crop CSI image to match RGB dimensions
                csi_height = gray_csi.shape[0]
                gray_csi = gray_csi[0:csi_height-crop_height, :]
                
                # Resize to match RGB image dimensions if needed
                target_height, target_width = rgb_image.shape[:2]
                if gray_csi.shape != (target_height, target_width):
                    gray_csi = cv2.resize(gray_csi, (target_width, target_height), interpolation=cv2.INTER_LINEAR)
                
                # Simulate depth based on intensity (brighter = closer)
                # Use more realistic depth range and scaling
                depth_normalized = (255 - gray_csi.astype(np.float32)) / 255.0
                depth_image = (depth_normalized * 30.0 * 1000).astype(np.uint16)  # 0-30m in mm
                return rgb_image, depth_image
        
        return None, None
    
    def get_intrinsics(self):
        """Get camera intrinsic parameters"""
        return self.intrinsics


class LiDARWrapper:
    """
    Wrapper for LiDAR sensor
    """
    def __init__(self, qcar_handle):
        self.qcar = qcar_handle
    
    def get_scan(self):
        """
        Get LiDAR scan data
        
        Returns:
            angles: Array of angles (radians)
            distances: Array of distances (meters)
        """
        success, angles, distances = self.qcar.get_lidar(samplePoints=360)
        if success:
            return angles, distances
        return None, None


def process_qcar_image_with_yolo(rgb_image, myYolo, depth_image=None , verbose=False):
    """Process RGB image with YOLO (image already obtained from sensors)"""
    if rgb_image is None:
        print("Failed: RGB image is None")
        return None, []
    
    # Run YOLO prediction - focus on cars (class 2)
    prediction = myYolo.predict(
        inputImg=rgb_image,
        classes=[2],  # Focus only on cars
        confidence=0.3,  # Lower confidence to detect more objects
        half=True,
        verbose=False
    )
    
    # Post-process results with enhanced information
    processedResults = myYolo.post_processing(
        alignedDepth=depth_image,
        clippingDistance=50.0,  # 50 meters clipping distance for depth
        verbose=verbose
    )
    
    # Get annotated image
    annotatedImg = myYolo.post_process_render(showFPS=True)
    if verbose:
        print(f"YOLO processing: {len(processedResults) if processedResults else 0} objects detected")
    
    return annotatedImg, processedResults


def rect_contains_point(bbox, point, image_width, image_height, shrink_factor=0.0):
    """
    Check if a 2D point lies within a bounding box, with optional shrinking
    Based on Early_Fusion's rectContains function
    
    Args:
        bbox: Bounding box [x1, y1, x2, y2] or [center_x, center_y, width, height]
        point: 2D point [x, y]
        image_width: Image width for normalization
        image_height: Image height for normalization  
        shrink_factor: Shrink box by this factor (0.0-1.0) to reduce outliers
        
    Returns:
        bool: True if point is inside the (potentially shrunk) bounding box
    """
    # Handle different bbox formats
    if hasattr(bbox, '__len__') and len(bbox) == 4:
        # Assume [x1, y1, x2, y2] format
        x1, y1, x2, y2 = bbox
        center_x = (x1 + x2) / 2.0
        center_y = (y1 + y2) / 2.0
        width = x2 - x1
        height = y2 - y1
    else:
        # Assume detection object with center and dimensions
        center_x = getattr(bbox, 'center_x', bbox.get('center_x', 0))
        center_y = getattr(bbox, 'center_y', bbox.get('center_y', 0)) 
        width = getattr(bbox, 'width', bbox.get('width', 0))
        height = getattr(bbox, 'height', bbox.get('height', 0))
    
    # Apply shrink factor to reduce outliers (Early_Fusion technique)
    effective_width = width * (1.0 - shrink_factor)
    effective_height = height * (1.0 - shrink_factor)
    
    # Calculate bounds
    x_min = center_x - effective_width / 2.0
    x_max = center_x + effective_width / 2.0
    y_min = center_y - effective_height / 2.0 
    y_max = center_y + effective_height / 2.0
    
    # Check if point is within bounds
    return (x_min <= point[0] <= x_max) and (y_min <= point[1] <= y_max)


def filter_outlier_distances(distances, method="one_sigma"):
    """
    Filter outlier distances using statistical methods from Early_Fusion
    
    Args:
        distances: Array of distance measurements
        method: "one_sigma", "three_sigma", or "iqr"
        
    Returns:
        filtered_distances: Array with outliers removed
    """
    if len(distances) < 3:
        return distances
        
    distances = np.array(distances)
    
    if method == "one_sigma":
        # Early_Fusion's one sigma method
        mean_dist = np.mean(distances)
        std_dist = np.std(distances) 
        mask = np.abs(distances - mean_dist) <= std_dist
        return distances[mask]
        
    elif method == "three_sigma": 
        # Three sigma rule
        mean_dist = np.mean(distances)
        std_dist = np.std(distances)
        mask = np.abs(distances - mean_dist) <= 3 * std_dist
        return distances[mask]
        
    elif method == "iqr":
        # Interquartile range method
        q1 = np.percentile(distances, 25)
        q3 = np.percentile(distances, 75)
        iqr = q3 - q1
        lower_bound = q1 - 1.5 * iqr
        upper_bound = q3 + 1.5 * iqr
        mask = (distances >= lower_bound) & (distances <= upper_bound)
        return distances[mask]
    
    return distances


def get_best_distance(distances, technique="median"):
    """
    Get best distance estimate using Early_Fusion techniques
    
    Args:
        distances: Array of distance measurements
        technique: "closest", "farthest", "average", "median"
        
    Returns:
        best_distance: Single distance estimate
    """
    if len(distances) == 0:
        return None
        
    distances = np.array(distances)
    
    if technique == "closest":
        return float(np.min(distances))
    elif technique == "farthest": 
        return float(np.max(distances))
    elif technique == "average":
        return float(np.mean(distances))
    elif technique == "median":
        return float(np.median(distances))
    else:
        return float(np.median(distances))  # Default to median


def fuse_lidar_with_detections(hQCar, detection_objects, geometry_manager, depth_image=None, image_width=640, image_height=480, verbose=False):
    """
    Enhanced Early_Fusion-style LiDAR-camera fusion for QCar2's 2D LiDAR
    Combines techniques from KITTI Early_Fusion with QCar2's 2D scanning LiDAR
    
    Args:
        hQCar: QCar handle for LiDAR data
        detection_objects: Enhanced YOLO detection objects
        geometry_manager: Robot geometry manager for coordinate transformations
        depth_image: Depth image from RealSense camera (optional but recommended)
        image_width: Camera image width
        image_height: Camera image height
        verbose: Enable debug output
        
    Returns:
        fused_objects: List of detection objects enhanced with accurate distance/angle data
    """
    # Initialize LiDAR-Camera calibration system
    lidar_cam_calibration = LiDARCameraCalibration(geometry_manager)
    
    # Get LiDAR data
    success, angles, distances = hQCar.get_lidar(samplePoints=360)
    if not success:
        if verbose:
            print("Debug: Failed to get LiDAR data")
        return []
    
    # Project LiDAR points to image plane (Early_Fusion technique)
    lidar_image_points, lidar_distances_fov, lidar_angles_fov = lidar_cam_calibration.get_lidar_in_image_fov(
        angles, distances, image_width, image_height, clip_distance=0.5
    )
    
    if verbose:
        print(f"Debug: LiDAR projected to image - {len(lidar_image_points)} points in FOV")
        if depth_image is not None:
            print(f"Debug: Depth image available - shape: {depth_image.shape}")
    
    fused_objects = []
    
    # Process each YOLO detection
    for i, detection in enumerate(detection_objects):
        if not hasattr(detection, 'center_x'):
            continue
            
        detection_distances = []
        depth_distances = []
        
        # Method 1: LiDAR point-in-bbox fusion (Early_Fusion style)
        for j, lidar_point in enumerate(lidar_image_points):
            # Check if LiDAR point falls within detection bbox (with shrink factor)
            if rect_contains_point(detection, lidar_point, image_width, image_height, shrink_factor=0.15):
                detection_distances.append(lidar_distances_fov[j])
        
        # Method 2: Depth camera extraction (if available)
        depth_distance = None
        if depth_image is not None:
            depth_distance = extract_depth_from_bbox(detection, depth_image, verbose=verbose)
            if depth_distance is not None:
                depth_distances.append(depth_distance)
        
        # Method 3: Fallback angular matching for 2D LiDAR
        if len(detection_distances) < 3:  # Not enough direct bbox matches
            pixel_angle = -(detection.center_x / image_width - 0.5) * geometry_manager.camera_fov_h
            angular_distances = find_lidar_distance_angular_match(
                angles, distances, pixel_angle, search_degrees=[3, 6, 10], verbose=verbose
            )
            detection_distances.extend(angular_distances)
        
        # Apply Early_Fusion outlier filtering
        if len(detection_distances) > 0:
            filtered_lidar_distances = filter_outlier_distances(detection_distances, method="one_sigma")
            best_lidar_distance = get_best_distance(filtered_lidar_distances, technique="median")
        else:
            best_lidar_distance = None
        
        # Fuse LiDAR and depth measurements
        final_distance, confidence_score = fuse_distance_measurements_early_fusion_style(
            best_lidar_distance, depth_distance, len(detection_distances), verbose=verbose
        )
        
        if final_distance is not None:
            # Calculate angle from pixel coordinates
            pixel_angle_deg = math.degrees(-(detection.center_x / image_width - 0.5) * geometry_manager.camera_fov_h)
            
            # Update detection with fusion results
            detection.distance = final_distance
            detection.angle = float(pixel_angle_deg)
            detection.lidar_distance = best_lidar_distance if best_lidar_distance else 0.0
            detection.depth_distance = depth_distance if depth_distance else 0.0
            detection.fusion_confidence = confidence_score
            detection.lidar_points_count = len(detection_distances)  # Early_Fusion diagnostic
            
            fused_objects.append(detection)
            
            if verbose:
                print(f"Debug: Early_Fusion style - Detection {i+1}")
                print(f"  - LiDAR points in bbox: {len(detection_distances)}")
                print(f"  - Final distance: {final_distance:.2f}m (confidence: {confidence_score:.2f})")
                print(f"  - LiDAR: {best_lidar_distance:.2f}m" if best_lidar_distance else "  - LiDAR: None")
                print(f"  - Depth: {depth_distance:.2f}m" if depth_distance else "  - Depth: None")
    
    return fused_objects


def find_lidar_distance_angular_match(angles, distances, target_angle, search_degrees=[3, 6, 10], verbose=False):
    """
    Find LiDAR distances using angular matching (fallback method)
    """
    matched_distances = []
    
    for search_deg in search_degrees:
        search_rad = math.radians(search_deg)
        angle_diff = np.abs(angles - target_angle)
        close_indices = np.where(angle_diff < search_rad)[0]
        
        if len(close_indices) > 0:
            close_distances = distances[close_indices]
            valid_distances = close_distances[(close_distances > 0.1) & (close_distances < 50)]
            
            if len(valid_distances) >= 2:
                matched_distances.extend(valid_distances)
                break  # Use tightest match that has enough points
    
    return matched_distances


def fuse_distance_measurements_early_fusion_style(lidar_distance, depth_distance, lidar_point_count, verbose=False):
    """
    Enhanced Early_Fusion style distance fusion with point count weighting
    
    Args:
        lidar_distance: Best LiDAR distance estimate
        depth_distance: Depth camera distance
        lidar_point_count: Number of LiDAR points used (affects confidence)
        verbose: Debug output
        
    Returns:
        final_distance: Fused distance estimate
        confidence_score: Confidence (0-1)
    """
    if lidar_distance is not None and depth_distance is not None:
        # Both measurements available - Early_Fusion weighted combination
        
        # Weight based on point count (more LiDAR points = higher confidence)
        lidar_confidence = min(1.0, lidar_point_count / 10.0)  # Max confidence at 10+ points
        depth_confidence = 0.7  # Depth camera baseline confidence
        
        # Distance-based weighting (Early_Fusion principle)
        if lidar_distance < 3.0:
            # Close range: prefer depth camera
            lidar_weight = 0.3 * lidar_confidence
            depth_weight = 0.7
        elif lidar_distance < 10.0:
            # Medium range: balanced
            lidar_weight = 0.6 * lidar_confidence  
            depth_weight = 0.4
        else:
            # Long range: prefer LiDAR
            lidar_weight = 0.8 * lidar_confidence
            depth_weight = 0.2
        
        # Normalize weights
        total_weight = lidar_weight + depth_weight
        lidar_weight /= total_weight
        depth_weight /= total_weight
        
        final_distance = lidar_weight * lidar_distance + depth_weight * depth_distance
        confidence_score = min(0.95, (lidar_confidence + depth_confidence) / 2.0)
        
        if verbose:
            print(f"    Early_Fusion: L={lidar_distance:.2f}m({lidar_weight:.2f}) + D={depth_distance:.2f}m({depth_weight:.2f}) = {final_distance:.2f}m")
        
        return final_distance, confidence_score
        
    elif lidar_distance is not None:
        # Only LiDAR available
        confidence_score = min(0.8, lidar_point_count / 15.0)  # Max 0.8 for LiDAR-only
        return lidar_distance, confidence_score
        
    elif depth_distance is not None:
        # Only depth available
        return depth_distance, 0.6  # Lower confidence for depth-only
        
    return None, 0.0


def extract_depth_from_bbox(detection, depth_image, verbose=False):
    """
    Extract depth information from detection bounding box
    
    Args:
        detection: YOLO detection object with bbox
        depth_image: Depth image (uint16, values in mm)
        verbose: Enable debug output
        
    Returns:
        mean_depth: Average depth in meters, or None if invalid
    """
    try:
        bbox = detection.bbox
        center_x, center_y = int(detection.center_x), int(detection.center_y)
        
        # Method 1: Sample depth at bbox center
        if (0 <= center_y < depth_image.shape[0] and 
            0 <= center_x < depth_image.shape[1]):
            center_depth = depth_image[center_y, center_x] / 1000.0  # Convert mm to meters
        else:
            center_depth = 0.0
        
        # Method 2: Average depth over bbox area (more robust)
        x1, y1, x2, y2 = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
        
        # Ensure bbox is within image bounds
        x1 = max(0, min(x1, depth_image.shape[1] - 1))
        x2 = max(0, min(x2, depth_image.shape[1] - 1))
        y1 = max(0, min(y1, depth_image.shape[0] - 1))
        y2 = max(0, min(y2, depth_image.shape[0] - 1))
        
        if x2 > x1 and y2 > y1:
            # Extract bbox region
            bbox_depths = depth_image[y1:y2, x1:x2]
            
            # Filter out invalid depths (too close/far or zero)
            valid_depths = bbox_depths[(bbox_depths > 500) & (bbox_depths < 50000)]  # 0.5m to 50m in mm
            
            if len(valid_depths) > 0:
                mean_depth = float(np.mean(valid_depths)) / 1000.0  # Convert to meters
                
                if verbose:
                    print(f"    Depth extraction: center={center_depth:.2f}m, bbox_mean={mean_depth:.2f}m, valid_pixels={len(valid_depths)}")
                
                # Use bbox average if we have enough valid pixels, otherwise use center
                if len(valid_depths) > 10:  # At least 10 valid pixels for reliable average
                    return mean_depth
                elif center_depth > 0.5 and center_depth < 50.0:  # Use center if valid
                    return center_depth
                    
        # Fallback to center depth if available
        if center_depth > 0.5 and center_depth < 50.0:
            return center_depth
            
    except Exception as e:
        if verbose:
            print(f"    Error extracting depth: {e}")
    
    return None


def find_lidar_distance_for_detection(detection, angles, distances, pixel_angle, verbose=False):
    """
    Find LiDAR distance for detection using improved angular matching
    
    Args:
        detection: YOLO detection object
        angles: LiDAR scan angles
        distances: LiDAR scan distances
        pixel_angle: Calculated pixel angle in radians
        verbose: Enable debug output
        
    Returns:
        median_distance: Median LiDAR distance in meters, or None if not found
    """
    # Progressive search: start with tight match, expand if needed
    search_angles = [math.radians(3), math.radians(6), math.radians(10), math.radians(15)]
    
    for search_angle in search_angles:
        angle_diff = np.abs(angles - pixel_angle)
        closest_indices = np.where(angle_diff < search_angle)[0]
        
        if len(closest_indices) > 0:
            closest_distances = distances[closest_indices]
            valid_distances = closest_distances[(closest_distances > 0.1) & (closest_distances < 50)]
            
            if len(valid_distances) >= 3:  # Need at least 3 points for reliable median
                median_distance = float(np.median(valid_distances))
                
                if verbose:
                    print(f"    LiDAR match: {len(valid_distances)} points within {math.degrees(search_angle):.1f}°, median={median_distance:.2f}m")
                
                return median_distance
    
    if verbose:
        print(f"    LiDAR: No reliable match found")
    
    return None


def fuse_distance_measurements(lidar_distance, depth_distance, verbose=False):
    """
    Fuse LiDAR and depth measurements using weighted combination
    
    Args:
        lidar_distance: Distance from LiDAR (meters)
        depth_distance: Distance from depth camera (meters)
        verbose: Enable debug output
        
    Returns:
        final_distance: Fused distance estimate
        confidence_score: Confidence in the measurement (0-1)
    """
    if lidar_distance is not None and depth_distance is not None:
        # Both measurements available - use weighted fusion
        # LiDAR is generally more accurate for longer ranges, depth camera for closer objects
        
        # Calculate weights based on distance and measurement reliability
        if lidar_distance < 5.0:  # Close range: depth camera more reliable
            lidar_weight = 0.4
            depth_weight = 0.6
        elif lidar_distance < 15.0:  # Medium range: both reliable, slight LiDAR preference
            lidar_weight = 0.6
            depth_weight = 0.4
        else:  # Long range: LiDAR much more reliable
            lidar_weight = 0.8
            depth_weight = 0.2
        
        # Check measurement consistency
        distance_diff = abs(lidar_distance - depth_distance)
        relative_diff = distance_diff / min(lidar_distance, depth_distance)
        
        if relative_diff < 0.3:  # Measurements agree (within 30%)
            final_distance = lidar_weight * lidar_distance + depth_weight * depth_distance
            confidence_score = 0.9  # High confidence
        elif relative_diff < 0.6:  # Moderate disagreement
             # Prefer the more reliable sensor based on range
            if lidar_distance > 10.0:
                final_distance = lidar_distance
                confidence_score = 0.7
            else:
                final_distance = depth_distance
                confidence_score = 0.6
        else:  # Large disagreement - use most reliable sensor
            if lidar_distance > 5.0:
                final_distance = lidar_distance
                confidence_score = 0.5
            else:
                final_distance = depth_distance
                confidence_score = 0.4
        
        if verbose:
            print(f"    Fusion: L={lidar_distance:.2f}m, D={depth_distance:.2f}m, Final={final_distance:.2f}m, Conf={confidence_score:.2f}")
            
    elif lidar_distance is not None:
        # Only LiDAR available
        final_distance = lidar_distance
        confidence_score = 0.7 if lidar_distance > 2.0 else 0.6  # Lower confidence for very close objects
        
        if verbose:
            print(f"    LiDAR only: {final_distance:.2f}m, Conf={confidence_score:.2f}")
            
    elif depth_distance is not None:
        # Only depth camera available
        final_distance = depth_distance
        confidence_score = 0.6 if depth_distance < 10.0 else 0.4  # Lower confidence for far objects
        
        if verbose:
            print(f"    Depth only: {final_distance:.2f}m, Conf={confidence_score:.2f}")
            
    else:
        # No measurements available
        final_distance = None
        confidence_score = 0.0
        
        if verbose:
            print(f"    No measurements available")
    
    return final_distance, confidence_score


def parse_arguments():
    """Parse command line arguments for coordinate frame setup"""
    parser = argparse.ArgumentParser(
        description='Semantic SLAM with Hybrid LiDAR+Depth Fusion',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Coordinate Frame Setup Options:
  1 = Default setup (current)
  2 = QCar-specific setup (recommended) 
  3 = Keep current and run calibration

Examples:
  python qcar2_tuto.py --setup 2                    # Use QCar-specific setup
  python qcar2_tuto.py --setup 3 --max-frames 500  # Run calibration, process 500 frames
  python qcar2_tuto.py --setup 1 --no-lidar-overlay # Default setup without LiDAR overlay
        """
    )
    
    parser.add_argument('--setup', type=int, choices=[1, 2, 3], default=1,
                       help='Coordinate frame setup choice (default: 1)')
    parser.add_argument('--max-frames', type=int, default=250,
                       help='Maximum number of frames to process (default: 250)')
    parser.add_argument('--no-lidar-overlay', action='store_true',
                       help='Disable LiDAR overlay on camera image')
    parser.add_argument('--verbose', action='store_true',
                       help='Enable verbose output for debugging')
    parser.add_argument('--show-maps', action='store_true',
                       help='Show occupancy grid visualization windows')
    
    return parser.parse_args()


def main():
    """
    Main function implementing semantic occupancy grid mapping
    """
    os.system('cls')
    
    # Parse command line arguments
    args = parse_arguments()
    
    print(f"=== Semantic SLAM with Hybrid LiDAR+Depth Fusion ===")
    print(f"Configuration:")
    print(f"  - Coordinate setup: {args.setup}")
    print(f"  - Max frames: {args.max_frames}")
    print(f"  - LiDAR overlay: {'OFF' if args.no_lidar_overlay else 'ON'}")
    print(f"  - Show maps: {'ON' if args.show_maps else 'OFF'}")
    print(f"  - Verbose mode: {'ON' if args.verbose else 'OFF'}")

    # Initialize geometry manager
    print("Initializing geometry manager...")
    geometry_manager = RobotGeometryManager()
    
    # Configure coordinate frame setup based on arguments
    run_calibration = False
    
    if args.setup == 2:
        print("Using QCar-specific coordinate frames...")
        geometry_manager.setup_coordinate_frames_qcar_specific()
    elif args.setup == 3:
        print("Will run calibration with current setup...")
        run_calibration = True
    else:
        print("Using default setup...")
    
    # --------- Communications with qlabs
    qlabs = QuanserInteractiveLabs()
    cv2.startWindowThread()
    
    print("=== Semantic Occupancy Grid Mapping System ===")
    print("Connecting to QLabs...")
    if not qlabs.open("localhost"):
        print("Unable to connect to QLabs")
        return    
    
    print("Connected to QLabs")
    qlabs.destroy_all_spawned_actors()
    
    # Set tutorial title
    hSystem = QLabsSystem(qlabs)
    hSystem.set_title_string('Semantic Occupancy Grid Mapping - YOLO + LiDAR + RGBD')
    
    ### Setup Environment and Sensors ###
    print("\n--- Setting up Environment ---")
    
    # Setup overview camera
    hCameraOverview = QLabsFreeCamera(qlabs)
    hCameraOverview.spawn_id(actorNumber=1, location=[-15.075, 26.703, 6.074], rotation=[0, 0.564, -1.586])
    
    # Spawn QCar with sensors (main robot)
    print("Spawning QCar with sensors...")
    hQCar = QLabsQCar2(qlabs)
    hQCar.spawn_id(actorNumber=0, location=[-8.700, 14.643, 0.005], rotation=[0,0,math.pi/2], waitForConfirmation=True)
    
    # Spawn target objects to detect
    print("Spawning target objects...")
    hQCar_Target1 = QLabsQCar2(qlabs)
    hQCar_Target1.spawn_id(actorNumber=2, location=[-11.048, 40.000, 0.005], rotation=[0,0,math.pi/2], waitForConfirmation=True)
    
    hQCar_Target2 = QLabsQCar2(qlabs)
    hQCar_Target2.spawn_id(actorNumber=3, location=[-1.848, 32.549, 0.005], rotation=[0,0,math.pi], waitForConfirmation=True)
    hQCar_Target2.set_led_strip_uniform(color=[0,1,0])  # Green for identification
    hQCar_Target2.set_velocity_and_request_state(forward=0.4, turn=0, headlights=True , leftTurnSignal=False, rightTurnSignal=False ,brakeSignal=False , reverseSignal=False)
    
    print("Environment setup complete")
    time.sleep(1)

    ### Initialize Semantic Mapping System ###
    print("\n--- Initializing Semantic Mapping System ---")
    
    # Initialize enhanced YOLO model
    myYolo = YOLOv8Wrapper_Huy(
        imageHeight=480,
        imageWidth=640,
    )
    
    # Initialize semantic occupancy grid
    semantic_ogm = SemanticOccupancyGrid(
        xLength=30,     # 30m x 30m area
        yLength=30,
        cellWidth=0.2,  # 20cm resolution
        pPrior=0.5
    )
    
    # Initialize sensor wrappers
    print("Initializing sensors...")
    realsense = RealSenseWrapper(hQCar)
    lidar = LiDARWrapper(hQCar)
    
    # Switch to RGB camera view for user
    hQCar.possess(hQCar.CAMERA_RGB)
    time.sleep(1)
    
    # Setup visualization windows
    print("Setting up visualization...")
    
    # LiDAR visualization
    lidar_app = pg.mkQApp("LiDAR Visualization")
    lidar_plot = pg.plot(title="LiDAR Data with Semantic Objects")
    lidar_plot.setXRange(-15, 15)
    lidar_plot.setYRange(-15, 15)
    lidar_data_plot = lidar_plot.plot([], [], pen=None, symbol='o', symbolBrush='r', symbolPen=None, symbolSize=2)
    semantic_objects_plot = lidar_plot.plot([], [], pen=None, symbol='s', symbolBrush='g', symbolPen=None, symbolSize=10)
    
    print("Enhanced Semantic SLAM with Hybrid LiDAR+Depth Fusion Ready!")
    print("Controls: 'l'=LiDAR overlay | 'h'=fusion stats | 'd'=depth info | 'q'=exit")
    print("Tip: Use --help for command-line options")
    
    ### Main Mapping Loop ###
    frame_count = 0
    robot_pose = [0.0, 0.0, 0.0]  # [x, y, theta] - relative to starting position
    all_detections = []
    
    # Configure visualization toggles based on arguments
    show_lidar_on_camera = not args.no_lidar_overlay  # Use argument setting
    show_maps = args.show_maps                        # Use argument setting
    show_semantic = True    # show semantic colored grid
    show_prob = False       # show raw occupancy probability colormap
    
    try:
        while True:
            frame_count += 1
            # Only print frame header occasionally (more frequent in verbose mode)
            if args.verbose or (frame_count % 10 == 1):
                print(f"Processing frame {frame_count}...")
            
            ### 1. Acquire Sensor Data ###
            
            # Get RGB and simulated depth images
            rgb_image_raw, depth_image = realsense.get_frames()
            if rgb_image_raw is None:
                print("Failed to get camera images")
                continue

            rgb_image = myYolo.pre_process(rgb_image_raw)  # Preprocess for YOLO input size
            # depth_image = myYolo.pre_process(depth_image_raw) if depth_image_raw is not None else None

            # Get LiDAR data
            angles, distances = lidar.get_scan()
            if angles is None:
                print("Failed to get LiDAR data")
                continue
            
            # Update robot pose in geometry manager
            geometry_manager.update_robot_pose(robot_pose[0], robot_pose[1], robot_pose[2])
            
            # Project LiDAR points to camera image coordinates
            projected_lidar_points, lidar_3d_points = geometry_manager.lidar_to_camera_projection(
                angles, distances, rgb_image.shape[1], rgb_image.shape[0]
            )
            
            ### 2. Enhanced YOLO Object Detection ###
            
            # Process with enhanced YOLO using the RGB image we already have
            annotated_image, yolo_detections = process_qcar_image_with_yolo(rgb_image, myYolo, depth_image, verbose=args.verbose)
            if annotated_image is None:
                continue
            
            # Print YOLO results based on verbosity
            if (args.verbose or frame_count % 10 == 1) and len(yolo_detections) > 0:
                print(f"YOLO detected {len(yolo_detections)} objects")
            
            ### 3. Sensor Fusion ###
            
            # Fuse LiDAR with enhanced YOLO detection objects using hybrid depth estimation
            fused_detections = fuse_lidar_with_detections(
                hQCar, yolo_detections, geometry_manager, 
                depth_image=depth_image,  # Pass depth image for hybrid fusion
                image_width=rgb_image.shape[1], 
                image_height=rgb_image.shape[0],
                verbose=args.verbose or (frame_count % 20 == 1)  # Verbose based on argument
            )
            
            # Use geometry manager to correct semantic object positions
            corrected_detections = geometry_manager.project_semantic_objects_to_lidar(
                fused_detections, rgb_image.shape[1], rgb_image.shape[0]
            )
            
            # Print fusion results based on verbosity
            if args.verbose or (frame_count % 5 == 1):
                print(f"Frame {frame_count}: {len(fused_detections)} fused, {len(corrected_detections)} corrected")
            
            # Run calibration if requested (only for first few frames with detections)
            if run_calibration and frame_count <= 20 and len(yolo_detections) > 0:
                print(f"\n--- Running Calibration (Frame {frame_count}) ---")
                angular_offset = geometry_manager.calibrate_camera_lidar_alignment(
                    yolo_detections, angles, distances, verbose=args.verbose
                )
                if abs(angular_offset) > math.radians(5):
                    print("Applying automatic correction...")
                    geometry_manager.adjust_camera_rotation(angular_offset)
                    run_calibration = False  # Only calibrate once

            ### 4. Update Semantic Occupancy Grid ( Not used yet )###

            # Update geometric occupancy with LiDAR
            semantic_ogm.update_with_lidar(
                x=robot_pose[0], 
                y=robot_pose[1], 
                theta=robot_pose[2], 
                angles=angles, 
                distances=distances
            )
            
            # Update semantic information with corrected YOLO detections
            for detection in corrected_detections:
                semantic_ogm.update_with_semantic_detection(detection)
                all_detections.append({
                    'frame': frame_count,
                    'detection': detection
                })
            
            # Update with depth information (if available)
            if depth_image is not None:
                semantic_ogm.update_with_depth(
                    depth_image=depth_image,
                    rgb_image=rgb_image,
                    camera_intrinsics=realsense.get_intrinsics(),
                    pose=robot_pose
                )
            
            ### 5. Visualization ###
            
            # Create base display image
            display_image = annotated_image.copy()
            
            # Draw LiDAR points on camera image if enabled
            if show_lidar_on_camera and len(projected_lidar_points) > 0:
                display_image = draw_lidar_on_camera_image(
                    display_image, projected_lidar_points, lidar_3d_points, 
                    detection_objects=corrected_detections,  # Pass detections to filter LiDAR points
                    proximity_threshold=80  # 80 pixel threshold for proximity to objects
                )
            
            # Add minimal frame info
            info_text = f"Frame {frame_count} | Objects: {len(corrected_detections)}"
            cv2.putText(display_image, info_text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            # Add simplified status (only show if depth not available)
            if depth_image is None:
                status_text = "Depth: OFF"
                cv2.putText(display_image, status_text, (10, display_image.shape[0] - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 100, 100), 1)
            
            # Simplified detection info - show only essential information
            for i, detection in enumerate(corrected_detections):
                if 'bbox' in detection and 'corrected_distance' in detection:
                    bbox = detection['bbox']
                    distance = detection['corrected_distance']
                    class_name = detection['class_name']
                    
                    # Position text above bbox - simplified
                    text_x = int(bbox[0])
                    text_y = int(max(bbox[1] - 10, 25))
                    
                    # Create clean, minimal info text
                    info_text = f"{class_name}: {distance:.1f}m"
                    cv2.putText(display_image, info_text, (text_x, text_y), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 2)  # White with black outline
                    cv2.putText(display_image, info_text, (text_x, text_y), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
            
            cv2.imshow('Semantic SLAM - Camera Feed', display_image)
            
            # Update LiDAR visualization with corrected semantic information
            if len(angles) > 0:
                x_lidar = np.sin(angles) * distances
                y_lidar = np.cos(angles) * distances
                lidar_data_plot.setData(x_lidar, y_lidar)
                
                # Highlight detected objects in LiDAR view using corrected positions
                if corrected_detections:
                    obj_x, obj_y = [], []
                    for detection in corrected_detections:
                        if 'lidar_x' in detection and 'lidar_y' in detection:
                            obj_x.append(detection['lidar_x'])
                            obj_y.append(detection['lidar_y'])
                            print(f"Object at LiDAR coordinates: ({detection['lidar_x']:.2f}, {detection['lidar_y']:.2f})")
                    semantic_objects_plot.setData(obj_x, obj_y)
                else:
                    semantic_objects_plot.setData([], [])
                
                QtWidgets.QApplication.instance().processEvents()
            
            # Occupancy / Semantic Grid Visualization (Not use yet)
            if show_maps and frame_count % 5 == 0:
                if show_semantic:
                    sem_img = build_semantic_overlay(semantic_ogm)
                    cv2.imshow('Semantic Occupancy Grid', sem_img)
                if show_prob:
                    prob_img = build_probability_colormap(semantic_ogm.get_occupancy_map())
                    if prob_img.shape[0] < 400:
                        prob_img = cv2.resize(prob_img, (prob_img.shape[1]*4, prob_img.shape[0]*4), interpolation=cv2.INTER_NEAREST)
                    cv2.imshow('Occupancy Probability Grid', prob_img)
            
            ### 6. Print Statistics ###
            if frame_count % 20 == 0:
                semantic_ogm.print_statistics()
                # Print enhanced detection summary
                if yolo_detections:
                    summary = myYolo.get_detection_summary()
                    print(f"Enhanced YOLO Summary: {summary}")
            
            ### 7. Handle User Input ###
            key = cv2.waitKey(30) & 0xFF
            if key == ord('q'):
                print("Stopping by user request...")
                break
            elif key == ord('l'):
                show_lidar_on_camera = not show_lidar_on_camera
                print(f"LiDAR camera overlay -> {'ON' if show_lidar_on_camera else 'OFF'}")
            elif key == ord('h'):
                # Show fusion statistics
                if len(fused_detections) > 0:
                    print(f"\n=== Hybrid Fusion Statistics (Frame {frame_count}) ===")
                    lidar_only = depth_only = hybrid = 0
                    total_confidence = 0
                    
                    for det in fused_detections:
                        if hasattr(det, 'lidar_distance') and hasattr(det, 'depth_distance'):
                            lidar_dist = det.lidar_distance
                            depth_dist = det.depth_distance
                            fusion_conf = getattr(det, 'fusion_confidence', 0.0)
                            total_confidence += fusion_conf
                            
                            if lidar_dist > 0 and depth_dist > 0:
                                hybrid += 1
                                print(f"  Hybrid: L={lidar_dist:.2f}m D={depth_dist:.2f}m Final={det.distance:.2f}m Conf={fusion_conf:.2f}")
                            elif lidar_dist > 0:
                                lidar_only += 1
                                print(f"  LiDAR only: {lidar_dist:.2f}m Conf={fusion_conf:.2f}")
                            elif depth_dist > 0:
                                depth_only += 1
                                print(f"  Depth only: {depth_dist:.2f}m Conf={fusion_conf:.2f}")
                    
                    avg_confidence = total_confidence / len(fused_detections) if len(fused_detections) > 0 else 0
                    print(f"Summary: {hybrid} hybrid, {lidar_only} LiDAR-only, {depth_only} depth-only")
                    print(f"Average confidence: {avg_confidence:.2f}")
                else:
                    print("No fused detections available for statistics")
            elif key == ord('d'):
                # Show depth image info
                if depth_image is not None:
                    print(f"\n=== Depth Image Information ===")
                    print(f"Shape: {depth_image.shape}, dtype: {depth_image.dtype}")
                    print(f"Min/Max values: {np.min(depth_image)}/{np.max(depth_image)}")
                    print(f"Mean depth: {np.mean(depth_image[depth_image > 0]) / 1000:.2f}m (excluding zeros)")
                    valid_pixels = np.sum(depth_image > 500)  # > 0.5m
                    total_pixels = depth_image.shape[0] * depth_image.shape[1]
                    print(f"Valid depth pixels: {valid_pixels}/{total_pixels} ({100*valid_pixels/total_pixels:.1f}%)")
                else:
                    print("No depth image available")
            elif key == ord('g'):
                show_maps = not show_maps
                print(f"Toggle maps -> {'ON' if show_maps else 'OFF'}")
            elif key == ord('s'):
                show_semantic = not show_semantic
                print(f"Semantic grid -> {'ON' if show_semantic else 'OFF'}")
            elif key == ord('o'):
                show_prob = not show_prob
                print(f"Probability grid -> {'ON' if show_prob else 'OFF'}")
            elif key == ord('c'):
                # Manual calibration
                if len(yolo_detections) > 0:
                    print("Running manual calibration...")
                    angular_offset = geometry_manager.calibrate_camera_lidar_alignment(
                        yolo_detections, angles, distances
                    )
                    if abs(angular_offset) > math.radians(2):
                        geometry_manager.adjust_camera_rotation(angular_offset)
                else:
                    print("No detections available for calibration")
            elif key == ord('v'):
                # Verify coordinate frames
                geometry_manager.verify_coordinate_frames()
            elif key == ord('r'):
                # Reset coordinate frames to QCar-specific
                print("Resetting to QCar-specific coordinate frames...")
                geometry_manager.setup_coordinate_frames_qcar_specific()
            
            # Small delay
            time.sleep(0.05)
            
            # Auto-stop after processing configured number of frames
            if frame_count >= args.max_frames:
                print(f"Reached maximum frame limit ({args.max_frames})")
                break
                
    except KeyboardInterrupt:
        print("\nStopped by keyboard interrupt")
    except Exception as e:
        print(f"Error in main loop: {e}")
        import traceback
        traceback.print_exc()
    
    ### Final Results and Cleanup ###
    print(f"\n=== Enhanced Semantic Mapping with Hybrid Fusion Complete ===")
    print(f"Total frames processed: {frame_count}")
    print(f"Total semantic detections: {len(all_detections)}")
    
    # Fusion performance analysis
    if all_detections:
        fusion_stats = {'hybrid': 0, 'lidar_only': 0, 'depth_only': 0, 'no_fusion': 0}
        confidence_scores = []
        
        for det_info in all_detections:
            detection = det_info['detection']
            if 'lidar_distance' in detection and 'depth_distance' in detection:
                lidar_dist = detection.get('lidar_distance', 0)
                depth_dist = detection.get('depth_distance', 0)
                fusion_conf = detection.get('fusion_confidence', 0)
                
                if lidar_dist > 0 and depth_dist > 0:
                    fusion_stats['hybrid'] += 1
                elif lidar_dist > 0:
                    fusion_stats['lidar_only'] += 1
                elif depth_dist > 0:
                    fusion_stats['depth_only'] += 1
                else:
                    fusion_stats['no_fusion'] += 1
                    
                if fusion_conf > 0:
                    confidence_scores.append(fusion_conf)
        
        print(f"\n=== Fusion Performance Analysis ===")
        print(f"Hybrid (LiDAR+Depth): {fusion_stats['hybrid']} detections")
        print(f"LiDAR only: {fusion_stats['lidar_only']} detections")
        print(f"Depth only: {fusion_stats['depth_only']} detections")
        print(f"No fusion: {fusion_stats['no_fusion']} detections")
        
        if confidence_scores:
            avg_confidence = np.mean(confidence_scores)
            print(f"Average fusion confidence: {avg_confidence:.3f}")
            print(f"Confidence std dev: {np.std(confidence_scores):.3f}")
    
    # Print final statistics
    semantic_ogm.print_statistics()
    
    # Summary of detected objects
    if all_detections:
        class_counts = defaultdict(int)
        for det_info in all_detections:
            class_name = det_info['detection'].get('class_name', 'unknown')
            class_counts[class_name] += 1
        
        print(f"\nDetected Object Summary:")
        for class_name, count in class_counts.items():
            print(f"  {class_name}: {count} detections")
    
    # Cleanup
    cv2.destroyAllWindows()
    time.sleep(2)

    hQCar_Target2.set_velocity_and_request_state(forward=0, turn=0, headlights=True , leftTurnSignal=False, rightTurnSignal=False ,brakeSignal=False , reverseSignal=False)

    qlabs.close()
    print("Enhanced Hybrid LiDAR+Depth Fusion SLAM System Shutdown Complete!")


if __name__ == "__main__":
    main()
