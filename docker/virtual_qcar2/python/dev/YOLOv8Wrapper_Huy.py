import cv2
import numpy as np
import torch
from pit.YOLO.nets import YOLOv8
from pit.YOLO.utils import TrafficLight, Obstacle

class EnhancedObstacle(Obstacle):
    """Enhanced Obstacle class with more detailed information"""
    def __init__(self, name='QCar', distance=0, conf=0, x=0, y=0):
        super().__init__(name, distance, conf, x, y)
        # Additional attributes
        self.bbox = None          # [x1, y1, x2, y2]
        self.center_x = None      # Center X coordinate
        self.center_y = None      # Center Y coordinate
        self.width = None         # Bounding box width
        self.height = None        # Bounding box height
        self.area = None          # Bounding box area
        self.class_id = None      # YOLO class ID

class EnhancedTrafficLight(TrafficLight):
    """Enhanced TrafficLight class with more detailed information"""
    def __init__(self, color='idle', distance=0):
        super().__init__(color, distance)
        # Additional attributes
        self.bbox = None
        self.center_x = None
        self.center_y = None
        self.width = None
        self.height = None
        self.area = None
        self.class_id = 9  # Traffic light class ID

class YOLOv8Wrapper_Huy(YOLOv8):
    """
    Enhanced YOLOv8 wrapper with improved post_processing method
    that provides more detailed detection information including:
    - Full bounding box coordinates
    - Center coordinates
    - Width, height, and area
    - Class ID information
    """
    
    def __init__(self, imageWidth=640, imageHeight=480, modelPath=None):
        """Initialize the enhanced YOLOv8 wrapper"""
        super().__init__(imageWidth, imageHeight, modelPath)
        print('YOLOv8Wrapper_Huy initialized with enhanced post-processing')
    
    def post_processing(self, alignedDepth=None, clippingDistance=10, verbose=False):
        """
        Enhanced post-processing method with detailed detection information.
        
        Args:
            alignedDepth (ndarray): A depth image aligned to the rgb input
            clippingDistance (float): Pixels in depth image further than this will be set to zero
            verbose (bool): Print debug information
            
        Returns:
            list: A list of Enhanced Obstacle/TrafficLight objects with detailed information
        """
        self.processedResults = []
        self._calc_distance = False

        if len(self.objectsDetected) == 0:
            if verbose:
                print("No objects detected")
            return self.processedResults
            
        # Get bounding boxes
        self.bounding = self.predictions[0].boxes.xyxy.cpu().numpy().astype(int)
        
        if verbose:
            print(f"Processing {len(self.objectsDetected)} detected objects")
        
        # Prepare depth processing if available
        if alignedDepth is not None:
            if alignedDepth.shape[:2] != (self.imageHeight, self.imageWidth):
                alignedDepth = cv2.resize(alignedDepth, (self.imageWidth, self.imageHeight))
            depth3D = np.dstack((alignedDepth, alignedDepth, alignedDepth))
            bgRemoved = np.where((depth3D > clippingDistance) | (depth3D <= 0), 0, depth3D)
            self._calc_distance = True
            self.depthTensor = torch.as_tensor(bgRemoved, device="cuda:0")
            
        # Process each detection
        for i in range(len(self.objectsDetected)):
            class_id = int(self.objectsDetected[i])
            
            # Get bounding box coordinates
            points = self.predictions[0].boxes.xyxy.cpu()[i]
            bbox = points.numpy().astype(int)  # [x1, y1, x2, y2]
            
            # Calculate center coordinates
            center_x = int((bbox[0] + bbox[2]) / 2)
            center_y = int((bbox[1] + bbox[3]) / 2)
            
            # Calculate dimensions
            width = bbox[2] - bbox[0]
            height = bbox[3] - bbox[1]
            area = width * height
            
            # Get confidence
            conf = self.predictions[0].boxes.conf.cpu().numpy()[i]
            
            if verbose:
                print(f"Object {i+1}: class_id={class_id}, bbox={bbox}, center=({center_x}, {center_y})")
            
            # Create appropriate object type
            if class_id == 9:  # Traffic light
                traffic_box = self.bounding[i]
                traffic_light_color = self.check_traffic_light(traffic_box, self.img)
                result = EnhancedTrafficLight(color=traffic_light_color)
                result.name += (' (' + traffic_light_color + ')')
                result.class_id = 9
            else:
                name = self.predictions[0].names[class_id]
                result = EnhancedObstacle(name=name)
                result.class_id = class_id
            
            # Set enhanced attributes
            result.bbox = bbox.tolist()
            result.center_x = center_x
            result.center_y = center_y
            result.width = width
            result.height = height
            result.area = area
            result.conf = float(conf)
            
            # Legacy attributes for compatibility
            result.x = bbox[0]  # Top-left x (for backward compatibility)
            result.y = bbox[1]  # Top-left y (for backward compatibility)
            
            # Calculate distance if depth is available
            if alignedDepth is not None and self.predictions[0].masks is not None:
                try:
                    mask = self.predictions[0].masks.data.cuda()[i]
                    distance = self.check_distance(mask, self.depthTensor[:, :, :1])
                    result.distance = distance.cpu().numpy().round(3)
                    if verbose:
                        print(f"  Distance: {result.distance}m")
                except Exception as e:
                    if verbose:
                        print(f"  Distance calculation failed: {e}")
                    result.distance = 0
            else:
                result.distance = 0
                
            self.processedResults.append(result)
            
        if verbose:
            print(f"Enhanced post-processing complete: {len(self.processedResults)} objects processed")
            
        return self.processedResults
    
    def get_detection_summary(self):
        """
        Get a summary of all detections
        
        Returns:
            dict: Summary information about detections
        """
        if not self.processedResults:
            return {"total_detections": 0}
            
        summary = {
            "total_detections": len(self.processedResults),
            "classes": {},
            "total_area": 0,
            "avg_confidence": 0
        }
        
        total_conf = 0
        for obj in self.processedResults:
            class_name = obj.name
            if class_name in summary["classes"]:
                summary["classes"][class_name] += 1
            else:
                summary["classes"][class_name] = 1
                
            summary["total_area"] += obj.area if obj.area else 0
            total_conf += obj.conf if obj.conf else 0
            
        if len(self.processedResults) > 0:
            summary["avg_confidence"] = total_conf / len(self.processedResults)
            