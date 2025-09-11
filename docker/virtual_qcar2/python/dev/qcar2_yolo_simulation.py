"""
QCar 2 YOLO Object Detection Example - Simulation

This example demonstrates how to use YOLOv8 object detection with QCar2 in simulation.
It captures images from the QCar2 RGB camera and performs object detection using YOLO.

Note: Make sure you have Quanser Interactive Labs open before running this example.
"""

import sys
import time
import math
import numpy as np
import cv2
import os

# QCar2 and QLabs imports
from qvl.qlabs import QuanserInteractiveLabs
from qvl.free_camera import QLabsFreeCamera
from qvl.qcar2 import QLabsQCar2
from qvl.system import QLabsSystem

# YOLO imports
from pit.YOLO.nets import YOLOv8

def main():
    os.system('cls')
    
    # Timing parameters
    sampleRate = 10.0  # Reduced from 30 for better visualization
    sampleTime = 1/sampleRate
    simulationTime = 60.0  # Run for 1 minute
    
    # Image parameters (QCar2 RGB camera resolution)
    imageWidth = 640
    imageHeight = 480
    
    print("Initializing QCar2 YOLO Object Detection...")
    
    # Initialize YOLO model
    print("Loading YOLO model...")
    myYolo = YOLOv8(
        imageHeight=imageHeight,
        imageWidth=imageWidth,
        # modelPath=r"C:\Users\Quang Huy Nugyen\Desktop\PHD_paper\Simulation\QCAR\Work\qcar2\docker\libraries\resources\pretrained_models\yolov8s-seg.pt"
    )
    print("YOLO model loaded successfully!")
    
    # Communications with QLabs
    qlabs = QuanserInteractiveLabs()
    cv2.startWindowThread()
    
    print("Connecting to QLabs...")
    if not qlabs.open("localhost"):
        print("Unable to connect to QLabs")
        return
    
    print("Connected to QLabs!")
    
    # Clean up any existing actors
    qlabs.destroy_all_spawned_actors()
    
    # Set title
    hSystem = QLabsSystem(qlabs)
    hSystem.set_title_string('QCar2 YOLO Object Detection')
    
    # Set up camera for overhead view
    hCameraOverview = QLabsFreeCamera(qlabs)
    hCameraOverview.spawn_id(actorNumber=1, location=[-15.075, 26.703, 6.074], rotation=[0, 0.564, -1.586])
    
    # Spawn QCar2
    print("Spawning QCar2...")
    hQCar = QLabsQCar2(qlabs)
    hQCar.spawn_id(actorNumber=0, location=[-8.700, 14.643, 0.005], rotation=[0, 0, math.pi/2])
    
    time.sleep(1)

    hQCar3 = QLabsQCar2(qlabs)

    hQCar3.spawn_id(actorNumber=3, location=[-10.605, 32.973, 0.005], rotation=[0,0,math.pi])

    
    print("QCar2 spawned successfully!")
    
    # Switch to QCar RGB camera view
    print("Switching to QCar RGB camera...")
    hQCar.possess(hQCar.CAMERA_RGB)
    time.sleep(1)
    
    # Turn on headlights for better visibility
    hQCar.set_velocity_and_request_state(forward=0, turn=0, headlights=True, 
                                        leftTurnSignal=False, rightTurnSignal=False, 
                                        brakeSignal=False, reverseSignal=False)
    
    print("Starting object detection loop...")
    print("Press 'q' to quit, 'c' to switch camera view")
    
    startTime = time.time()
    frame_count = 0
    camera_view = 0  # 0 = RGB camera view, 1 = overhead view
    
    try:
        while (time.time() - startTime) < simulationTime:
            loop_start = time.time()
            
            # Get image from QCar RGB camera
            success, camera_image = hQCar.get_image(camera=hQCar.CAMERA_RGB)
            
            if success and camera_image is not None:
                # Preprocess image for YOLO
                rgbProcessed = myYolo.pre_process(camera_image)
                
                # Run YOLO prediction
                prediction = myYolo.predict(
                    inputImg=rgbProcessed,
                    classes=[0, 1, 2, 3, 5, 7, 9, 11],  # person, bicycle, car, motorcycle, bus, truck, traffic light, stop sign
                    confidence=0.5,
                    half=True,
                    verbose=False
                )
                
                # Post-process without depth information (simulation only)
                processedResults = myYolo.post_processing(
                    alignedDepth=None,  # No depth needed for simulation
                    clippingDistance=None
                )
                
                # Print detected objects
                if processedResults:
                    print(f"Frame {frame_count}: Detected {len(processedResults)} objects:")
                    for obj in processedResults:
                        if hasattr(obj, '__dict__'):
                            print(f"  - {obj.__dict__}")
                        else:
                            print(f"  - {obj}")
                    print('---------------------------')
                
                # Render annotated image
                annotatedImg = myYolo.post_process_render(showFPS=True)
                
                # Display the result
                cv2.imshow('QCar2 YOLO Object Detection', annotatedImg)
                
                frame_count += 1
            else:
                print("Failed to get image from QCar camera")
                # Show a black image if camera fails
                black_img = np.zeros((imageHeight, imageWidth, 3), dtype=np.uint8)
                cv2.putText(black_img, 'Camera Error', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.imshow('QCar2 YOLO Object Detection', black_img)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("Quit requested by user")
                break
            elif key == ord('c'):
                # Switch camera view
                camera_view = 1 - camera_view
                if camera_view == 0:
                    hQCar.possess(hQCar.CAMERA_RGB)
                    print("Switched to QCar RGB camera")
                else:
                    hCameraOverview.possess()
                    print("Switched to overhead camera")
                time.sleep(0.5)
            elif key == ord('w'):
                # Move forward
                hQCar.set_velocity_and_request_state(forward=1, turn=0, headlights=True, 
                                                    leftTurnSignal=False, rightTurnSignal=False, 
                                                    brakeSignal=False, reverseSignal=False)
            elif key == ord('s'):
                # Move backward
                hQCar.set_velocity_and_request_state(forward=-1, turn=0, headlights=True, 
                                                    leftTurnSignal=False, rightTurnSignal=False, 
                                                    brakeSignal=True, reverseSignal=True)
            elif key == ord('a'):
                # Turn left while moving
                hQCar.set_velocity_and_request_state(forward=1, turn=math.pi/6, headlights=True, 
                                                    leftTurnSignal=True, rightTurnSignal=False, 
                                                    brakeSignal=False, reverseSignal=False)
            elif key == ord('d'):
                # Turn right while moving
                hQCar.set_velocity_and_request_state(forward=1, turn=-math.pi/6, headlights=True, 
                                                    leftTurnSignal=False, rightTurnSignal=True, 
                                                    brakeSignal=False, reverseSignal=False)
            elif key == ord(' '):
                # Stop
                hQCar.set_velocity_and_request_state(forward=0, turn=0, headlights=True, 
                                                    leftTurnSignal=False, rightTurnSignal=False, 
                                                    brakeSignal=True, reverseSignal=False)
            
            # Calculate timing for consistent frame rate
            loop_end = time.time()
            computationTime = loop_end - loop_start
            sleepTime = sampleTime - (computationTime % sampleTime)
            
            if sleepTime > 0:
                time.sleep(sleepTime)
                
    except KeyboardInterrupt:
        print("User interrupted!")
        
    finally:
        # Stop the car and close connections
        print("Stopping QCar and cleaning up...")
        try:
            hQCar.set_velocity_and_request_state(forward=0, turn=0, headlights=False, 
                                                leftTurnSignal=False, rightTurnSignal=False, 
                                                brakeSignal=False, reverseSignal=False)
        except:
            pass
            
        cv2.destroyAllWindows()
        qlabs.close()
        print("Done! Processed {} frames".format(frame_count))


if __name__ == "__main__":
    print("QCar2 YOLO Object Detection")
    print("Controls:")
    print("  W - Move forward")
    print("  S - Move backward") 
    print("  A - Turn left")
    print("  D - Turn right")
    print("  SPACE - Stop")
    print("  C - Switch camera view")
    print("  Q - Quit")
    print()
    main()
