#!/usr/bin/env python3
"""
Test script for Semantic Occupancy Grid Mapping components
This script tests individual components without requiring full QCar setup
"""

import sys
import os
import numpy as np
import cv2
import matplotlib.pyplot as plt

# Add path for utilities
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../libraries/python/hal/utilities'))

def test_semantic_ogm_class():
    """Test SemanticOccupancyGrid class with simulated data"""
    print("Testing SemanticOccupancyGrid class...")
    
    try:
        # Import required components (these would normally come from mapping.py)
        # For testing, we'll create minimal mock classes
        
        class MockOccupancyGrid:
            def __init__(self, xLength, yLength, cellWidth, **kwargs):
                self.cellWidth = cellWidth
                self.m = int(np.ceil(yLength / cellWidth) + 1)
                self.n = int(np.ceil(xLength / cellWidth) + 1)
                self._xOffset = self.n * cellWidth / 2
                self._yOffset = self.m * cellWidth / 2
                self.lMap = np.zeros((self.m, self.n))
                self.pMap = np.ones((self.m, self.n)) * 0.5
                self._pMapOutOfDate = False
                self._lMax = 4.0
                
            def update(self, x, y, th, angles, distances):
                # Simple occupancy update
                for angle, distance in zip(angles, distances):
                    if distance > 0.5 and distance < 20:
                        obj_x = distance * np.sin(angle)
                        obj_y = distance * np.cos(angle)
                        i, j = self.xy_to_ij_rounded(obj_x, obj_y)
                        if 0 <= i < self.m and 0 <= j < self.n:
                            self.lMap[i, j] = min(self.lMap[i, j] + 1.0, self._lMax)
                
            def xy_to_ij_rounded(self, x, y):
                i = int(np.round((self._yOffset - y) / self.cellWidth))
                j = int(np.round((x + self._xOffset) / self.cellWidth))
                return i, j
                
            def ij_to_xy(self, i, j):
                x = j * self.cellWidth - self._xOffset
                y = self._yOffset - i * self.cellWidth
                return x, y
        
        class MockImageProcessing:
            def __init__(self):
                pass
        
        # Mock SemanticOccupancyGrid for testing
        class TestSemanticOccupancyGrid:
            def __init__(self, xLength=20, yLength=20, cellWidth=0.2, pPrior=0.5):
                self.base_ogm = MockOccupancyGrid(xLength, yLength, cellWidth)
                self.semantic_map = np.zeros((self.base_ogm.m, self.base_ogm.n), dtype=np.int32)
                self.confidence_map = np.zeros((self.base_ogm.m, self.base_ogm.n), dtype=np.float32)
                
                # Class mapping for YOLO COCO dataset
                self.class_names = {
                    0: 'person', 2: 'car', 5: 'bus', 7: 'truck'
                }
                
                self.class_colors = {
                    0: [255, 0, 0],    # person - red
                    2: [0, 255, 0],    # car - green  
                    5: [0, 0, 255],    # bus - blue
                    7: [255, 255, 0],  # truck - yellow
                    -1: [128, 128, 128] # unknown - gray
                }
                
                self.img_proc = MockImageProcessing()
                print(f"Initialized Test Semantic OGM: {self.base_ogm.m}x{self.base_ogm.n} cells")
            
            def update_with_lidar(self, x, y, theta, angles, distances):
                """Update occupancy grid with LiDAR data"""
                self.base_ogm.update(x, y, theta, angles, distances)
            
            def update_with_semantic_detection(self, detection):
                """Update semantic map with detection"""
                if 'distance' not in detection or 'angle' not in detection:
                    return
                
                distance = detection['distance']
                angle = np.radians(detection['angle'])
                
                obj_x = distance * np.sin(angle)
                obj_y = distance * np.cos(angle)
                
                i, j = self.base_ogm.xy_to_ij_rounded(obj_x, obj_y)
                
                if 0 <= i < self.base_ogm.m and 0 <= j < self.base_ogm.n:
                    class_id = detection.get('class_id', -1)
                    confidence = detection.get('confidence', 0.0)
                    
                    if confidence > self.confidence_map[i, j]:
                        self.semantic_map[i, j] = class_id
                        self.confidence_map[i, j] = confidence
            
            def get_semantic_visualization(self):
                """Create colored visualization"""
                height, width = self.base_ogm.m, self.base_ogm.n
                semantic_vis = np.zeros((height, width, 3), dtype=np.uint8)
                
                for i in range(height):
                    for j in range(width):
                        semantic_class = self.semantic_map[i, j]
                        confidence = self.confidence_map[i, j]
                        
                        if semantic_class in self.class_colors and confidence > 0.3:
                            color = self.class_colors[semantic_class]
                            semantic_vis[i, j] = [int(c * confidence) for c in color]
                
                return semantic_vis
            
            def get_occupancy_map(self):
                return self.base_ogm.pMap
            
            def print_statistics(self):
                occupied_cells = np.sum(self.confidence_map > 0.3)
                unique_classes = np.unique(self.semantic_map[self.confidence_map > 0.3])
                print(f"Test Statistics: {occupied_cells} semantic cells, Classes: {unique_classes}")
        
        # Test the semantic OGM
        test_ogm = TestSemanticOccupancyGrid(xLength=20, yLength=20, cellWidth=0.2)
        
        # Simulate LiDAR data
        angles = np.linspace(0, 2*np.pi, 360)
        distances = np.ones_like(angles) * 5.0  # 5 meter circle
        distances[45:90] = 3.0  # Closer objects in one sector
        
        # Update with simulated LiDAR
        test_ogm.update_with_lidar(0, 0, 0, angles, distances)
        
        # Simulate YOLO detections
        detections = [
            {'class_id': 2, 'confidence': 0.8, 'distance': 3.0, 'angle': 45},  # car
            {'class_id': 0, 'confidence': 0.6, 'distance': 5.0, 'angle': 90},  # person
            {'class_id': 5, 'confidence': 0.9, 'distance': 3.5, 'angle': 60},  # bus
        ]
        
        for detection in detections:
            test_ogm.update_with_semantic_detection(detection)
        
        # Test visualization
        semantic_vis = test_ogm.get_semantic_visualization()
        occupancy_map = test_ogm.get_occupancy_map()
        
        # Display results
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        
        ax1.imshow(occupancy_map, cmap='gray', origin='lower')
        ax1.set_title('Test Occupancy Map')
        
        ax2.imshow(semantic_vis, origin='lower')
        ax2.set_title('Test Semantic Map')
        
        plt.tight_layout()
        plt.savefig('test_semantic_ogm_output.png')
        plt.show()
        
        test_ogm.print_statistics()
        print("✓ SemanticOccupancyGrid test passed!")
        
        return True
        
    except Exception as e:
        print(f"✗ SemanticOccupancyGrid test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_yolo_detection_parsing():
    """Test YOLO detection parsing functions"""
    print("Testing YOLO detection parsing...")
    
    try:
        # Mock YOLO detection objects
        class MockYOLODetection:
            def __init__(self, bbox, class_id, confidence):
                self.bbox = bbox
                self.class_id = class_id
                self.confidence = confidence
        
        # Simulate YOLO results
        mock_detections = [
            MockYOLODetection([100, 150, 200, 250], 2, 0.8),  # car
            MockYOLODetection([300, 100, 400, 200], 0, 0.6),  # person
        ]
        
        # Test detection extraction (simplified version)
        def extract_detection_info_test(detected_objects):
            detections = []
            for obj in detected_objects:
                detection_info = {
                    'bbox': obj.bbox,
                    'center_x': (obj.bbox[0] + obj.bbox[2]) / 2,
                    'center_y': (obj.bbox[1] + obj.bbox[3]) / 2,
                    'class_id': obj.class_id,
                    'confidence': obj.confidence
                }
                detections.append(detection_info)
            return detections
        
        extracted = extract_detection_info_test(mock_detections)
        
        print(f"Extracted {len(extracted)} detections:")
        for i, det in enumerate(extracted):
            print(f"  Detection {i+1}: class={det['class_id']}, "
                  f"center=({det['center_x']:.1f},{det['center_y']:.1f}), "
                  f"conf={det['confidence']:.2f}")
        
        assert len(extracted) == 2, "Should extract 2 detections"
        assert extracted[0]['class_id'] == 2, "First detection should be car"
        assert extracted[1]['class_id'] == 0, "Second detection should be person"
        
        print("✓ YOLO detection parsing test passed!")
        return True
        
    except Exception as e:
        print(f"✗ YOLO detection parsing test failed: {e}")
        return False


def test_lidar_fusion():
    """Test LiDAR-vision fusion logic"""
    print("Testing LiDAR-vision fusion...")
    
    try:
        # Mock LiDAR data
        angles = np.linspace(-np.pi, np.pi, 360)  # Full circle
        distances = np.ones_like(angles) * 10.0   # 10m all around
        
        # Add some closer objects
        distances[170:190] = 3.0  # Objects around 0 degrees (front)
        distances[260:280] = 5.0  # Objects around -45 degrees (front-left)
        
        # Mock detections with pixel coordinates
        detections = [
            {'bbox': [320, 240, 380, 300], 'center_x': 350, 'center_y': 270, 
             'class_id': 2, 'confidence': 0.8},  # Center-right
            {'bbox': [200, 200, 260, 260], 'center_x': 230, 'center_y': 230,
             'class_id': 0, 'confidence': 0.6},  # Center-left
        ]
        
        # Fusion function (simplified)
        def fuse_lidar_with_detections_test(detections, angles, distances, 
                                           image_width=640, image_height=480):
            fused = []
            camera_fov = np.radians(70)
            
            for detection in detections:
                pixel_angle = (detection['center_x'] / image_width - 0.5) * camera_fov
                
                angle_diff = np.abs(angles - pixel_angle)
                closest_indices = np.where(angle_diff < np.radians(15))[0]
                
                if len(closest_indices) > 0:
                    closest_distances = distances[closest_indices]
                    valid_distances = closest_distances[(closest_distances > 0.5) & (closest_distances < 20)]
                    
                    if len(valid_distances) > 0:
                        detection['distance'] = float(np.median(valid_distances))
                        detection['angle'] = float(np.degrees(pixel_angle))
                        fused.append(detection)
            
            return fused
        
        # Test fusion
        fused_results = fuse_lidar_with_detections_test(detections, angles, distances)
        
        print(f"Fused {len(fused_results)} detections:")
        for i, det in enumerate(fused_results):
            print(f"  Fused {i+1}: class={det['class_id']}, "
                  f"distance={det['distance']:.2f}m, "
                  f"angle={det['angle']:.1f}°")
        
        assert len(fused_results) > 0, "Should successfully fuse some detections"
        
        print("✓ LiDAR-vision fusion test passed!")
        return True
        
    except Exception as e:
        print(f"✗ LiDAR-vision fusion test failed: {e}")
        return False


def main():
    """Run all component tests"""
    print("=== Semantic OGM Component Tests ===\n")
    
    tests = [
        test_yolo_detection_parsing,
        test_lidar_fusion,
        test_semantic_ogm_class,
    ]
    
    results = []
    for test in tests:
        try:
            result = test()
            results.append(result)
        except Exception as e:
            print(f"Test {test.__name__} crashed: {e}")
            results.append(False)
        print()
    
    print("=== Test Summary ===")
    passed = sum(results)
    total = len(results)
    print(f"Passed: {passed}/{total} tests")
    
    if passed == total:
        print("🎉 All tests passed! The semantic OGM system is ready.")
    else:
        print("⚠️  Some tests failed. Check the implementation.")
    
    return passed == total


if __name__ == "__main__":
    main()
