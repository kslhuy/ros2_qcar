import sys
import threading
import time
import cv2
import numpy as np
from qvl.qlabs import QuanserInteractiveLabs
from qvl.qcar2 import QLabsQCar2
from qvl.basic_shape import QLabsBasicShape
from pit.YOLO.nets import YOLOv8
from qvl.traffic_light import QLabsTrafficLight

def main():
    # Initialize QLabs connection
    qlabs = QuanserInteractiveLabs()
    print("Connecting to QLabs...")
    if not qlabs.open("localhost"):
        print("Failed to connect to QLabs")
        return
    print("Connected")

    # Clear any existing actors
    qlabs.destroy_all_spawned_actors()

    # Spawn QCar 2
    qcar = QLabsQCar2(qlabs)
    qcar.spawn_id(actorNumber=0, location=[0, 0, 0], rotation=[0, 0, 0], scale=[1,1,1])

    qcar_2 = QLabsQCar2(qlabs)
    qcar_2.spawn_id(actorNumber=1, location=[8, 0, 0], rotation=[0, 0, 0], scale=[1,1,1])

    trafficLight1 = QLabsTrafficLight(qlabs)
    trafficLight1.spawn_id_degrees(actorNumber=1, location=[15, -3, 0], rotation=[0,0,-90], scale=[1, 1, 1], configuration=0, waitForConfirmation=False)
    lock = threading.Lock()
    
    def light():
        intersection1Flag = 0

        print('Starting Traffic Light Sequence')

        while(True):
            with lock:
                #intersection 1

                if intersection1Flag == 0:
                    trafficLight1.set_color(color=QLabsTrafficLight.COLOR_RED)

                if intersection1Flag == 1:
                    trafficLight1.set_color(color=QLabsTrafficLight.COLOR_GREEN)

                if intersection1Flag == 2:
                    trafficLight1.set_color(color=QLabsTrafficLight.COLOR_YELLOW)

                intersection1Flag = (intersection1Flag + 1)%4

            time.sleep(5)
            
    thread = threading.Thread(target=light)
    thread.start()
    
    time.sleep(1)

    yolo = YOLOv8()
    yolo2 = YOLOv8()
    try:
        while True:
            with lock:
                # Request and process camera data
                success, image_data = qcar.get_image(camera=qcar.CAMERA_RGB)
                success2, image_depth_data = qcar.get_image(camera=qcar.CAMERA_DEPTH)
                if success and image_data is not None and success2 and image_depth_data is not None:
                    # Convert image data to OpenCV format
                    image = yolo.pre_process(image_data)
                    image_depth_data = yolo2.pre_process(image_depth_data)
                    yolo.predict(image)
                    image = yolo.render()
                    yolo.post_processing(alignedDepth=image_depth_data)
                    image_post_processing = yolo.post_process_render()
                    # image = image_data
                    if image is None:
                        print("Failed to decode image")
                        continue

                    # # Convert to HSV for color-based detection
                    # hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                    # # Green color range for cube detection
                    # lower_green = np.array([35, 100, 100])
                    # upper_green = np.array([85, 255, 255])
                    # mask = cv2.inRange(hsv, lower_green, upper_green)

                    # # Find contours
                    # contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    # for contour in contours:
                    #     if cv2.contourArea(contour) > 500:  # Filter small contours
                    #         x, y, w, h = cv2.boundingRect(contour)
                    #         cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    #         cv2.putText(image, "Green Cube", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    #         print(f"Detected Green Cube at ({x}, {y})")

                    # Display the processed image
                    cv2.imshow("render", image)
                    cv2.imshow("post process render", image_post_processing)
                    cv2.imshow("depth image", image_depth_data)

                # Press 'q' to exit
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    except KeyboardInterrupt:
        print("Program interrupted")

    finally:
        # Clean up
        qlabs.destroy_all_spawned_actors()
        qlabs.close()
        cv2.destroyAllWindows()
        print("QLabs connection closed")

if __name__ == "__main__":
    main()