"""
QCar 2 YOLO Object Detection - Simulation Version

This is a simplified version that closely follows the hardware YOLO example 
but adapted for QCar2 simulation using get_image() instead of QCar2DepthAligned.

Note: Make sure you have Quanser Interactive Labs open before running this example.
"""

import numpy as np
import time
import cv2
import math
import os
import sys

# QCar2 and QLabs imports
from qvl.qlabs import QuanserInteractiveLabs
from qvl.free_camera import QLabsFreeCamera
from qvl.qcar2 import QLabsQCar2
from qvl.system import QLabsSystem

# YOLO imports
from pit.YOLO.nets import YOLOv8

## Timing Parameters and methods 
def elapsed_time():
    return time.time() - startTime

# Parameters
sampleRate = 10.0  # Reduced for better visualization
sampleTime = 1/sampleRate
simulationTime = 30.0
print('Sample Time: ', sampleTime)

# Image parameters
imageWidth = 640
imageHeight = 480

# Initialize QLabs connection
def initialize_qlabs_and_qcar():
    """Initialize QLabs connection and spawn QCar2"""
    qlabs = QuanserInteractiveLabs()
    
    print("Connecting to QLabs...")
    if not qlabs.open("localhost"):
        print("Unable to connect to QLabs")
        return None, None
    
    print("Connected!")
    qlabs.destroy_all_spawned_actors()
    
    # Set title
    hSystem = QLabsSystem(qlabs)
    hSystem.set_title_string('QCar2 YOLO Object Detection - Simulation')
    
    # Spawn QCar2
    hQCar = QLabsQCar2(qlabs)
    success = hQCar.spawn_id(actorNumber=0, location=[-8.700, 14.643, 0.005], 
                            rotation=[0, 0, math.pi/2], waitForConfirmation=True)
    
    if not success:
        print("Failed to spawn QCar2")
        qlabs.close()
        return None, None
    
    # Turn on headlights and set up camera
    hQCar.set_velocity_and_request_state(forward=0, turn=0, headlights=True, 
                                        leftTurnSignal=False, rightTurnSignal=False, 
                                        brakeSignal=False, reverseSignal=False)
    
    # Switch to RGB camera
    hQCar.possess(hQCar.CAMERA_RGB)
    time.sleep(0.5)
    
    return qlabs, hQCar

# Get RGB image from QCar2
def get_qcar_image(hQCar):
    """Get RGB image from QCar2 simulation"""
    success, camera_image = hQCar.get_image(camera=hQCar.CAMERA_RGB)
    
    if success and camera_image is not None:
        return camera_image
    else:
        return None

# Initialize YOLOv8 segmentation model
myYolo = YOLOv8(
    # modelPath = 'path/to/model', 
    imageHeight=imageHeight,
    imageWidth=imageWidth,
)

# Initialize QLabs and QCar
qlabs, hQCar = initialize_qlabs_and_qcar()

if qlabs is None or hQCar is None:
    print("Failed to initialize QLabs or QCar2")
    sys.exit(1)

print("Starting YOLO object detection loop...")
print("Press any key in the OpenCV window to exit")

try:
    startTime = time.time()
    frame_count = 0
    
    while elapsed_time() < simulationTime:
        start = time.time()

        # Get RGB image from QCar2
        rgb_image = get_qcar_image(hQCar)
        
        if rgb_image is not None:
            # Preprocess for YOLO
            rgbProcessed = myYolo.pre_process(rgb_image)
            
            # Run YOLO prediction
            prediction = myYolo.predict(
                inputImg=rgbProcessed,
                classes=[2, 9, 11],  # car, traffic light, stop sign
                confidence=0.3,
                half=True,
                verbose=False
            )
            
            # Post-process results (no depth information for simulation)
            processedResults = myYolo.post_processing(
                alignedDepth=None,  # No depth needed for simulation
                clippingDistance=5
            )
            
            # Print detected objects
            if processedResults:
                for obj in processedResults:
                    if hasattr(obj, '__dict__'):
                        print(obj.__dict__)
                print('---------------------------')

            # Render annotated image
            annotatedImg = myYolo.post_process_render(showFPS=True)
            cv2.imshow('QCar2 YOLO Object Detection - Simulation', annotatedImg)
            
            frame_count += 1
        else:
            print("Failed to get image from QCar camera")
            # Show error message
            error_img = np.zeros((imageHeight, imageWidth, 3), dtype=np.uint8)
            cv2.putText(error_img, 'No Camera Feed', (200, 240), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.imshow('QCar2 YOLO Object Detection - Simulation', error_img)

        # End timing this iteration
        end = time.time()

        # Calculate the computation time, and the time that the thread should pause/sleep for
        computationTime = end - start
        sleepTime = sampleTime - (computationTime % sampleTime)

        # Pause/sleep for sleepTime in milliseconds
        msSleepTime = int(1000 * sleepTime)
        if msSleepTime <= 0:
            msSleepTime = 1
        
        # Check for key press to exit
        key = cv2.waitKey(msSleepTime) & 0xFF
        if key != 255:  # Any key pressed
            break

except KeyboardInterrupt:
    print("User interrupted!")
    
finally:
    print("Cleaning up...")
    try:
        # Stop the car
        hQCar.set_velocity_and_request_state(forward=0, turn=0, headlights=False, 
                                            leftTurnSignal=False, rightTurnSignal=False, 
                                            brakeSignal=False, reverseSignal=False)
    except:
        pass
    
    cv2.destroyAllWindows()
    qlabs.close()
    print(f"Done! Processed {frame_count} frames")
