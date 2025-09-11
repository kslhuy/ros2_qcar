# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : File Description and Imports

"""
vehicle_control.py

Skills acivity code for vehicle control lab guide.
Students will implement a vehicle speed and steering controller.
Please review Lab Guide - vehicle control PDF
"""
import os
import signal
import numpy as np
from threading import Thread
import time
import cv2
import pyqtgraph as pg
import matplotlib.pyplot as plt


from pal.products.qcar import QCar, QCarGPS, IS_PHYSICAL_QCAR
from pal.utilities.scope import MultiScope
from pal.utilities.math import wrap_to_pi
from hal.content.qcar_functions import QCarEKF
from hal.products.mats import SDCSRoadMap
import pal.resources.images as images

# Import QCarRealSense from the docker libraries
import sys
# sys.path.append(r'c:\Users\Quang Huy Nugyen\Desktop\PHD_paper\Simulation\QCAR\Work\qcar2\docker\libraries\python')
from hal.content.qcar import QCarRealSense

#===================== QLabs Setup ========================
# This modified version of vehicle control is intended to be used in
# the Development Container with the Virtual QCar. 
# 
# It is intended to be run after you have spawned the QCar in the 
# diagonal portion of the SDCS roadmap or the diagonal portion of
# the Cityscape.


#================ Experiment Configuration ================
# ===== Timing Parameters
# - tf: experiment duration in seconds.
# - startDelay: delay to give filters time to settle in seconds.
# - controllerUpdateRate: control update rate in Hz. Shouldn't exceed 500
tf = 3000
startDelay = 1
controllerUpdateRate = 100

# ===== Speed Controller Parameters
# - v_ref: desired velocity in m/s
# - K_p: proportional gain for speed controller
# - K_i: integral gain for speed controller
v_ref = 0.3
K_p = 0.1
K_i = 1

# ===== Steering Controller Parameters
# - enableSteeringControl: whether or not to enable steering control
# - K_stanley: K gain for stanley controller
# - nodeSequence: list of nodes from roadmap. Used for trajectory generation.
enableSteeringControl = True
K_stanley = 1
nodeSequence = [10, 4, 20, 13,10]

# ===== RealSense Camera Parameters
# - enableRealSense: whether to enable RealSense camera
enableRealSense = True

#endregion
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : Initial setup
if enableSteeringControl:
    roadmap = SDCSRoadMap(leftHandTraffic=False)
    # # print(dir(roadmap))

    
    # # Plot all nodes
    # for i in range(len(roadmap.nodes)):  # Or len(roadmap.nodes)
    #     pose = roadmap.get_node_pose(i).squeeze()
    #     plt.plot(pose[0], pose[1], 'bo')  # blue dot
    #     plt.text(pose[0], pose[1], str(i), fontsize=9)

    # # Optionally: draw connections (if you know them or can access .edges or .adjacency)
    # # Example: if roadmap.edges = [(0,1), (1,2)]
    # # for a, b in roadmap.edges:
    # #     pose_a = roadmap.get_node_pose(a).squeeze()
    # #     pose_b = roadmap.get_node_pose(b).squeeze()
    # #     plt.plot([pose_a[0], pose_b[0]], [pose_a[1], pose_b[1]], 'k--')

    # plt.axis('equal')
    # plt.title('SDCS Road Map Nodes')
    # plt.xlabel('X')
    # plt.ylabel('Y')
    # plt.grid(True)
    # plt.show()

    waypointSequence = roadmap.generate_path(nodeSequence)
    initialPose = roadmap.get_node_pose(nodeSequence[0]).squeeze()
    print("Initial Pose:", initialPose)

else:
    initialPose = [0, 0, 0]

calibrate=False

# Define the calibration pose
# Calibration pose is either [0,0,-pi/2] or [0,2,-pi/2]
# Comment out the one that is not used 
#calibrationPose = [0,0,-np.pi/2]
calibrationPose = [0,2,-np.pi/2]

# Used to enable safe keyboard triggered shutdown
global KILL_THREAD
KILL_THREAD = False

follower_path = []

def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True
signal.signal(signal.SIGINT, sig_handler)
#endregion

class SpeedController:

    def __init__(self, kp=0, ki=0):
        self.maxThrottle = 0.3

        self.kp = kp
        self.ki = ki

        self.ei = 0


    # ==============  SECTION A -  Speed Control  ====================
    def update(self, v, v_ref, dt):
        
        e = v_ref - v
        self.ei += dt*e

        return np.clip(
            self.kp*e + self.ki*self.ei,
            -self.maxThrottle,
            self.maxThrottle
        )
    
        return 0

class SteeringController:

    def __init__(self, waypoints, k=1, cyclic=True):
        self.maxSteeringAngle = np.pi/6

        self.wp = waypoints
        self.N = len(waypoints[0, :])
        self.wpi = 0

        self.k = k
        self.cyclic = cyclic

        self.p_ref = (0, 0)
        self.th_ref = 0

    # ==============  SECTION B -  Steering Control  ====================
    def update(self, p, th, speed):
        wp_1 = self.wp[:, np.mod(self.wpi, self.N-1)]
        wp_2 = self.wp[:, np.mod(self.wpi+1, self.N-1)]
        
        v = wp_2 - wp_1
        v_mag = np.linalg.norm(v)
        try:
            v_uv = v / v_mag
        except ZeroDivisionError:
            return 0

        tangent = np.arctan2(v_uv[1], v_uv[0])

        s = np.dot(p-wp_1, v_uv)

        if s >= v_mag:
            if  self.cyclic or self.wpi < self.N-2:
                self.wpi += 1

        ep = wp_1 + v_uv*s
        ct = ep - p
        dir = wrap_to_pi(np.arctan2(ct[1], ct[0]) - tangent)

        ect = np.linalg.norm(ct) * np.sign(dir)
        psi = wrap_to_pi(tangent-th)

        self.p_ref = ep
        self.th_ref = tangent

        return np.clip(
            wrap_to_pi(psi + np.arctan2(self.k*ect, speed)),
            -self.maxSteeringAngle,
            self.maxSteeringAngle)
        

def controlLoop():
    #region controlLoop setup
    global KILL_THREAD, enableRealSense
    u = 0
    delta = 0
    # used to limit data sampling to 10hz
    countMax = controllerUpdateRate / 10
    count = 0
    #endregion

    #region Controller initialization
    speedController = SpeedController(
        kp=K_p,
        ki=K_i
    )
    if enableSteeringControl:
        steeringController = SteeringController(
            waypoints=waypointSequence,
            k=K_stanley
        )
    #endregion

    #region QCar interface setup
    qcar = QCar(readMode=1, frequency=controllerUpdateRate)
    if enableSteeringControl or calibrate:
        ekf = QCarEKF(x_0=initialPose)
        gps = QCarGPS(initialPose=calibrationPose,calibrate=calibrate)
    else:
        gps = memoryview(b'')
    
    # Initialize RealSense camera if enabled
    if enableRealSense:
        try:
            realsense = QCarRealSense(
                mode='RGB&DEPTH',
                frameWidthRGB=640,
                frameHeightRGB=480,
                frameRateRGB=30,
                frameWidthDepth=640,
                frameHeightDepth=480,
                frameRateDepth=15
            )
            print("QCarRealSense initialized successfully")
        except Exception as e:
            print(f"Failed to initialize QCarRealSense: {e}")
            enableRealSense = False
            realsense = None
    #endregion

    with qcar, gps:
        # Open RealSense camera in context manager if enabled
        if enableRealSense and realsense is not None:
            with realsense:
                t0 = time.time()
                t=0
                while (t < tf+startDelay) and (not KILL_THREAD):
                    #region : Loop timing update
                    tp = t
                    t = time.time() - t0
                    dt = t-tp
                    #endregion

                    #region : Read from sensors and update state estimates
                    qcar.read()
                    
                    # Read from RealSense camera
                    rgb_timestamp = realsense.read_RGB()
                    depth_timestamp = realsense.read_depth('PX')
                    
                    if rgb_timestamp > 0 and depth_timestamp > 0:  # Both frames available
                        rgb_image = realsense.imageBufferRGB
                        depth_image = realsense.imageBufferDepthPX
                        
                        # Display images in real-time (every 5th frame to reduce CPU load)
                        if count % 5 == 0:
                            # Display RGB image
                            rgb_display = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
                            cv2.imshow('QCar RealSense - RGB', rgb_display)
                            
                            # Display depth image (normalize for better visibility)
                            depth_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                            depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
                            cv2.imshow('QCar RealSense - Depth', depth_colored)
                            
                            cv2.waitKey(1)  # Process GUI events
                    if enableSteeringControl:
                        if gps.readGPS():
                            y_gps = np.array([
                                gps.position[0],
                                gps.position[1],
                                gps.orientation[2]
                            ])
                            ekf.update(
                                [qcar.motorTach, delta],
                                dt,
                                y_gps,
                                qcar.gyroscope[2],
                            )
                        else:
                            ekf.update(
                                [qcar.motorTach, delta],
                                dt,
                                None,
                                qcar.gyroscope[2],
                            )

                        x = ekf.x_hat[0,0]
                        y = ekf.x_hat[1,0]
                        th = ekf.x_hat[2,0]
                        p = ( np.array([x, y])
                            + np.array([np.cos(th), np.sin(th)]) * 0.2)
                        
                        # ---- Follower Path Logging ----
                        follower_path.append([p[0], p[1]])
                        if len(follower_path) > 1000:  # Prevent overflow
                            follower_path.pop(0)

                    v = qcar.motorTach
                    # print('v',v)
                    #endregion

                    #region : Update controllers and write to car
                    if t < startDelay:
                        u = 0
                        delta = 0
                    else:
                        #region : Speed controller update
                        u = speedController.update(v, v_ref, dt)
                        # print('u',u)
                        #endregion

                        #region : Steering controller update
                        if enableSteeringControl:
                            delta = steeringController.update(p, th, v)
                        else:
                            delta = 0
                        #endregion

                    qcar.write(u, delta)
                    #endregion

                    #region : Update Scopes
                    count += 1
                    if count >= countMax and t > startDelay:
                        
                        t_plot = t - startDelay

                        if len(follower_path) > 1:
                            xs, ys = zip(*follower_path)
                            followerPath.setData(xs, ys)

                        # Speed control scope
                        speedScope.axes[0].sample(t_plot, [v, v_ref])
                        speedScope.axes[1].sample(t_plot, [v_ref-v])
                        speedScope.axes[2].sample(t_plot, [u])

                        # Steering control scope
                        if enableSteeringControl:
                            steeringScope.axes[4].sample(t_plot, [[p[0],p[1]]])

                            p[0] = ekf.x_hat[0,0]
                            p[1] = ekf.x_hat[1,0]

                            x_ref = steeringController.p_ref[0]
                            y_ref = steeringController.p_ref[1]
                            th_ref = steeringController.th_ref

                            x_ref = gps.position[0]
                            y_ref = gps.position[1]
                            th_ref = gps.orientation[2]

                            steeringScope.axes[0].sample(t_plot, [p[0], x_ref])
                            steeringScope.axes[1].sample(t_plot, [p[1], y_ref])
                            steeringScope.axes[2].sample(t_plot, [th, th_ref])
                            steeringScope.axes[3].sample(t_plot, [delta])

                            arrow.setPos(p[0], p[1])
                            arrow.setStyle(angle=180-th*180/np.pi)

                        count = 0
                    #endregion
                    
                # out of while loop : Stop the car
                qcar.read_write_std(throttle= 0, steering= 0)
        else:
            # Run without RealSense
            t0 = time.time()
            t=0
            while (t < tf+startDelay) and (not KILL_THREAD):
                #region : Loop timing update
                tp = t
                t = time.time() - t0
                dt = t-tp
                #endregion

                #region : Read from sensors and update state estimates
                qcar.read()
                if enableSteeringControl:
                    if gps.readGPS():
                        y_gps = np.array([
                            gps.position[0],
                            gps.position[1],
                            gps.orientation[2]
                        ])
                        ekf.update(
                            [qcar.motorTach, delta],
                            dt,
                            y_gps,
                            qcar.gyroscope[2],
                        )
                    else:
                        ekf.update(
                            [qcar.motorTach, delta],
                            dt,
                            None,
                            qcar.gyroscope[2],
                        )

                    x = ekf.x_hat[0,0]
                    y = ekf.x_hat[1,0]
                    th = ekf.x_hat[2,0]
                    p = ( np.array([x, y])
                        + np.array([np.cos(th), np.sin(th)]) * 0.2)
                    
                    # ---- Follower Path Logging ----
                    follower_path.append([p[0], p[1]])
                    if len(follower_path) > 1000:  # Prevent overflow
                        follower_path.pop(0)

                v = qcar.motorTach
                # print('v',v)
                #endregion

                #region : Update controllers and write to car
                if t < startDelay:
                    u = 0
                    delta = 0
                else:
                    #region : Speed controller update
                    u = speedController.update(v, v_ref, dt)
                    # print('u',u)
                    #endregion

                    #region : Steering controller update
                    if enableSteeringControl:
                        delta = steeringController.update(p, th, v)
                    else:
                        delta = 0
                    #endregion

                qcar.write(u, delta)
                #endregion

                #region : Update Scopes
                count += 1
                if count >= countMax and t > startDelay:
                    
                    t_plot = t - startDelay

                    if len(follower_path) > 1:
                        xs, ys = zip(*follower_path)
                        followerPath.setData(xs, ys)

                    # Speed control scope
                    speedScope.axes[0].sample(t_plot, [v, v_ref])
                    speedScope.axes[1].sample(t_plot, [v_ref-v])
                    speedScope.axes[2].sample(t_plot, [u])

                    # Steering control scope
                    if enableSteeringControl:
                        steeringScope.axes[4].sample(t_plot, [[p[0],p[1]]])

                        p[0] = ekf.x_hat[0,0]
                        p[1] = ekf.x_hat[1,0]

                        x_ref = steeringController.p_ref[0]
                        y_ref = steeringController.p_ref[1]
                        th_ref = steeringController.th_ref

                        x_ref = gps.position[0]
                        y_ref = gps.position[1]
                        th_ref = gps.orientation[2]

                        steeringScope.axes[0].sample(t_plot, [p[0], x_ref])
                        steeringScope.axes[1].sample(t_plot, [p[1], y_ref])
                        steeringScope.axes[2].sample(t_plot, [th, th_ref])
                        steeringScope.axes[3].sample(t_plot, [delta])

                        arrow.setPos(p[0], p[1])
                        arrow.setStyle(angle=180-th*180/np.pi)

                    count = 0
                #endregion
                
            # out of while loop : Stop the car
            qcar.read_write_std(throttle= 0, steering= 0)

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : Setup and run experiment
if __name__ == '__main__':

    #region : Setup scopes
    if IS_PHYSICAL_QCAR:
        fps = 10
    else:
        fps = 30
    # Scope for monitoring speed controller
    speedScope = MultiScope(
        rows=3,
        cols=1,
        title='Vehicle Speed Control',
        fps=fps
    )
    speedScope.addAxis(
        row=0,
        col=0,
        timeWindow=10,#tf
        yLabel='Vehicle Speed [m/s]',
        yLim=(0, 1)
    )
    speedScope.axes[0].attachSignal(name='v_meas', width=2)
    speedScope.axes[0].attachSignal(name='v_ref')

    speedScope.addAxis(
        row=1,
        col=0,
        timeWindow=10,
        yLabel='Speed Error [m/s]',
        yLim=(-0.5, 0.5)
    )
    speedScope.axes[1].attachSignal()

    speedScope.addAxis(
        row=2,
        col=0,
        timeWindow=10,
        xLabel='Time [s]',
        yLabel='Throttle Command [%]',
        yLim=(-0.3, 0.3)
    )
    speedScope.axes[2].attachSignal()

    # Scope for monitoring steering controller
    if enableSteeringControl:
        steeringScope = MultiScope(
            rows=4,
            cols=2,
            title='Vehicle Steering Control',
            fps=fps
        )

        steeringScope.addAxis(
            row=0,
            col=0,
            timeWindow=10,
            yLabel='x Position [m]',
            yLim=(-2.5, 2.5)
        )
        steeringScope.axes[0].attachSignal(name='x_meas')
        steeringScope.axes[0].attachSignal(name='x_ref')

        steeringScope.addAxis(
            row=1,
            col=0,
            timeWindow=10,
            yLabel='y Position [m]',
            yLim=(-1, 5)
        )
        steeringScope.axes[1].attachSignal(name='y_meas')
        steeringScope.axes[1].attachSignal(name='y_ref')
        # Heading angle
        steeringScope.addAxis(
            row=2,
            col=0,
            timeWindow=10,
            yLabel='Heading Angle [rad]',
            yLim=(-3.5, 3.5)
        )
        steeringScope.axes[2].attachSignal(name='th_meas')
        steeringScope.axes[2].attachSignal(name='th_ref')

        steeringScope.addAxis(
            row=3,
            col=0,
            timeWindow=10,
            yLabel='Steering Angle [rad]',
            yLim=(-0.6, 0.6)
        )
        steeringScope.axes[3].attachSignal()
        steeringScope.axes[3].xLabel = 'Time [s]'

        steeringScope.addXYAxis(
            row=0,
            col=1,
            rowSpan=4,
            xLabel='x Position [m]',
            yLabel='y Position [m]',
            xLim=(-2.5, 2.5),
            yLim=(-1, 5)
        )

        im = cv2.imread(
            images.SDCS_CITYSCAPE,
            cv2.IMREAD_GRAYSCALE
        )

        steeringScope.axes[4].attachImage(
            scale=(-0.002035, 0.002035),
            offset=(1125,2365),
            rotation=180,
            levels=(0, 255)
        )
        steeringScope.axes[4].images[0].setImage(image=im)

        referencePath = pg.PlotDataItem(
            pen={'color': (85,168,104), 'width': 2},
            name='Reference'
        )
        steeringScope.axes[4].plot.addItem(referencePath)
        referencePath.setData(waypointSequence[0, :],waypointSequence[1, :])

        followerPath = pg.PlotDataItem(
            pen={'color': (255, 255, 0), 'width': 2},  # Yellow path
            name='Follower'
        )
        steeringScope.axes[4].plot.addItem(followerPath)


        steeringScope.axes[4].attachSignal(name='Estimated', width=2)

        steeringScope.axes[4].attachSignal(name='Follower', width=2, color='y')


        arrow = pg.ArrowItem(
            angle=180,
            tipAngle=60,
            headLen=10,
            tailLen=10,
            tailWidth=5,
            pen={'color': 'w', 'fillColor': [196,78,82], 'width': 1},
            brush=[196,78,82]
        )
        arrow.setPos(initialPose[0], initialPose[1])
        steeringScope.axes[4].plot.addItem(arrow)

    # Setup image display window for RealSense if enabled
    if enableRealSense:
        cv2.namedWindow('QCar RealSense - RGB', cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow('QCar RealSense - Depth', cv2.WINDOW_AUTOSIZE)
        print("RealSense display windows created")
    #endregion

    #region : Setup control thread, then run experiment
    controlThread = Thread(target=controlLoop)
    controlThread.start()

    try:
        while controlThread.is_alive() and (not KILL_THREAD):
            MultiScope.refreshAll()
            time.sleep(0.01)
    finally:
        KILL_THREAD = True
        # Clean up OpenCV windows
        if enableRealSense:
            cv2.destroyAllWindows()

    input('Experiment complete. Press any key to exit...')
    #endregion
#endregion