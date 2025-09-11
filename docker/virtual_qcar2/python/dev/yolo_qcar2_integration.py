"""
Minimal QCar2 YOLO Integration Example

This shows how to add YOLO object detection to an existing QCar2 simulation.
Just add this code to your existing QCar2 script after spawning the car.
"""

import cv2
import numpy as np
from pit.YOLO.nets import YOLOv8

def setup_yolo(imageWidth=640, imageHeight=480):
    """Initialize YOLO model for QCar2"""
    myYolo = YOLOv8(
        imageHeight=imageHeight,
        imageWidth=imageWidth,
    )
    print("YOLO model loaded!")
    return myYolo

def process_qcar_image_with_yolo(hQCar, myYolo):
    """Get image from QCar2 and process with YOLO"""
    # Get RGB image from QCar2
    success, camera_image = hQCar.get_image(camera=hQCar.CAMERA_CSI_BACK)
    
    if not success or camera_image is None:
        return None, []
    
    # Preprocess image for YOLO
    rgbProcessed = myYolo.pre_process(camera_image)
    
    # Run YOLO prediction
    prediction = myYolo.predict(
        inputImg=rgbProcessed,
        classes=[0, 1, 2, 3, 5, 7, 9, 11],  # person, bicycle, car, motorcycle, bus, truck, traffic light, stop sign
        confidence=0.3,
        half=True,
        verbose=False
    )
    
    # Post-process results (no depth needed for simulation)
    processedResults = myYolo.post_processing(
        alignedDepth=None,
        clippingDistance=None
    )
    
    # Get annotated image
    annotatedImg = myYolo.post_process_render(showFPS=True)
    
    return annotatedImg, processedResults

# Example usage - add this to your existing QCar2 script:
"""
# After spawning hQCar2, add these lines:

# Initialize YOLO
myYolo = setup_yolo()

# Switch to RGB camera
hQCar2.possess(hQCar2.CAMERA_RGB)
time.sleep(0.5)

# Main loop for object detection
for i in range(100):  # Run for 100 frames
    annotated_image, detected_objects = process_qcar_image_with_yolo(hQCar2, myYolo)
    
    if annotated_image is not None:
        # Display the annotated image
        cv2.imshow('YOLO Detection', annotated_image)
        
        # Print detected objects
        if detected_objects:
            print(f"Frame {i}: Detected {len(detected_objects)} objects")
            for obj in detected_objects:
                if hasattr(obj, '__dict__'):
                    print(f"  - {obj.__dict__}")
        
        # Wait for key press (1ms timeout)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    time.sleep(0.1)  # Small delay

cv2.destroyAllWindows()
"""
