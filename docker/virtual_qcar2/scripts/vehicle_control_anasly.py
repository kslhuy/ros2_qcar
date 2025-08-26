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
import statistics


from pal.products.qcar import QCar, QCarGPS, IS_PHYSICAL_QCAR
from pal.utilities.scope import MultiScope
from pal.utilities.math import wrap_to_pi
from hal.content.qcar_functions import QCarEKF
from hal.products.mats import SDCSRoadMap
import pal.resources.images as images

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
tf = 60
startDelay = 1
controllerUpdateRate = 100

# ===== Performance Monitoring Parameters
# - enablePerformanceLogging: whether to enable detailed timing logs
# - timingLogInterval: how often to log timing details (every N loops)
# - enableGPSTimingAnalysis: whether to analyze GPS call performance
# - enablePositionLogging: whether to log position and rotation data
# - positionLogInterval: how often to log position/rotation (every N loops)
enableFinalPerformanceLogging = True

enablePerformanceLogging = False
timingLogInterval = 200  # Log every 200 loops
enableGPSTimingAnalysis = True
enablePositionLogging = True
positionLogInterval = 100  # Log position every 100 loops

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
    initialPose = [-1.205667, -0.826762, -0.718744]
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
    global KILL_THREAD
    u = 0
    delta = 0
    # used to limit data sampling to 10hz
    countMax = controllerUpdateRate / 10
    count = 0
    
    # Performance timing variables
    loop_count = 0
    timing_log_interval = timingLogInterval  # Use configuration parameter
    gps_times = []  # Store GPS times for analysis
    
    # Position and rotation tracking
    initial_position_logged = False
    initial_ekf_position = None
    initial_gps_position = None
    initial_rotation = None
    position_history = []  # Store position history for analysis
    gps_position_history = []  # Store GPS position history
    
    # GPS availability tracking
    gps_available_count = 0  # How many times GPS was available
    gps_unavailable_count = 0  # How many times GPS was unavailable
    gps_availability_history = []  # Track GPS availability over time
    first_gps_time = None  # When first GPS data was received
    last_gps_time = None   # When last GPS data was received
    
    # Detailed performance tracking
    all_loop_times = []
    all_sensor_times = []
    all_control_times = []
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
    #endregion

    with qcar, gps:
        t0 = time.time()
        t=0
        while (t < tf+startDelay) and (not KILL_THREAD):
            #region : Loop timing update
            loop_start_time = time.perf_counter()  # High precision timing for loop measurement
            tp = t
            t = time.time() - t0
            dt = t-tp
            loop_count += 1
            #endregion

            #region : Read from sensors and update state estimates
            sensor_start_time = time.perf_counter()
            qcar.read()
            sensor_read_time = (time.perf_counter() - sensor_start_time) * 1000  # Convert to ms
            
            gps_time = 0.0
            ekf_time = 0.0
            
            if enableSteeringControl:
                # GPS data collection timing
                gps_start_time = time.perf_counter()
                gps_available = gps.readGPS()
                gps_read_time = (time.perf_counter() - gps_start_time) * 1000
                gps_time = gps_read_time
                
                # Track GPS availability
                if gps_available:
                    gps_available_count += 1
                    if first_gps_time is None:
                        first_gps_time = t
                    last_gps_time = t
                    gps_availability_history.append([t, True])  # Time and availability status
                else:
                    gps_unavailable_count += 1
                    gps_availability_history.append([t, False])
                
                # Keep GPS availability history manageable
                if len(gps_availability_history) > 2000:
                    gps_availability_history.pop(0)
                
                # Store GPS times for analysis (only when GPS is available)
                if enableGPSTimingAnalysis and gps_available:
                    gps_times.append(gps_read_time)
                    if len(gps_times) > 100:  # Keep last 100 measurements
                        gps_times.pop(0)
                
                # EKF update timing
                ekf_start_time = time.perf_counter()
                if gps_available:
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
                ekf_time = (time.perf_counter() - ekf_start_time) * 1000
                gps_time = gps_read_time

                x = ekf.x_hat[0,0]
                y = ekf.x_hat[1,0]
                th = ekf.x_hat[2,0]
                p = ( np.array([x, y])
                    + np.array([np.cos(th), np.sin(th)]) * 0.2)
                
                # ---- Position and Rotation Logging ----
                # Log initial positions once
                if not initial_position_logged and gps_available:
                    initial_ekf_position = [x, y, th]
                    initial_gps_position = [gps.position[0], gps.position[1], gps.orientation[2]]
                    initial_rotation = [gps.orientation[0], gps.orientation[1], gps.orientation[2]]
                    initial_position_logged = True
                    
                    print(f"\n=== INITIAL POSITIONS CAPTURED ===")
                    print(f"Initial EKF Position: ({initial_ekf_position[0]:.6f}, {initial_ekf_position[1]:.6f}, {initial_ekf_position[2]:.6f})")
                    print(f"Initial GPS Position: ({initial_gps_position[0]:.6f}, {initial_gps_position[1]:.6f}, {initial_gps_position[2]:.6f})")
                    print(f"Initial GPS Rotation: (roll={initial_rotation[0]:.6f}, pitch={initial_rotation[1]:.6f}, yaw={initial_rotation[2]:.6f})")
                    print(f"Expected Initial Pose: {initialPose}")
                    print(f"Calibration Pose: {calibrationPose}")
                
                # Store position history for analysis
                if enablePositionLogging:
                    position_history.append([x, y, th, t])
                    if gps_available:
                        gps_position_history.append([gps.position[0], gps.position[1], gps.orientation[2], t])
                    
                    # Keep only last 1000 entries to prevent memory issues
                    if len(position_history) > 1000:
                        position_history.pop(0)
                    if len(gps_position_history) > 1000:
                        gps_position_history.pop(0)
                
                # ---- Follower Path Logging ----
                follower_path.append([p[0], p[1]])
                if len(follower_path) > 1000:  # Prevent overflow
                    follower_path.pop(0)


            v = qcar.motorTach
            # print('v',v)
            #endregion

            #region : Update controllers and write to car
            control_start_time = time.perf_counter()
            
            if t < startDelay:
                u = 0
                delta = 0
            else:
                #region : Speed controller update
                speed_control_start = time.perf_counter()
                u = speedController.update(v, v_ref, dt)
                speed_control_time = (time.perf_counter() - speed_control_start) * 1000
                # print('u',u)
                #endregion

                #region : Steering controller update
                steering_control_start = time.perf_counter()
                if enableSteeringControl:
                    delta = steeringController.update(p, th, v)
                else:
                    delta = 0
                steering_control_time = (time.perf_counter() - steering_control_start) * 1000
                #endregion

            # Write commands to vehicle
            write_start_time = time.perf_counter()
            qcar.write(u, delta)
            write_time = (time.perf_counter() - write_start_time) * 1000
            
            control_time = (time.perf_counter() - control_start_time) * 1000
            #endregion

            #region : Update Scopes
            scope_start_time = time.perf_counter()
            
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
            
            scope_time = (time.perf_counter() - scope_start_time) * 1000
            #endregion
            
            #region : Performance Logging
            # Calculate total loop time
            total_loop_time = (time.perf_counter() - loop_start_time) * 1000
            
            # Store performance data for analysis
            all_loop_times.append(total_loop_time)
            all_sensor_times.append(sensor_read_time)
            all_control_times.append(control_time)
            
            # Keep only last 500 measurements to prevent memory issues
            if len(all_loop_times) > 500:
                all_loop_times.pop(0)
                all_sensor_times.pop(0)
                all_control_times.pop(0)
            
            # Target loop time for the configured update rate
            target_loop_time = 1000.0 / controllerUpdateRate  # Convert Hz to ms
            
            # Log timing information periodically
            if enablePerformanceLogging and loop_count % timing_log_interval == 0 and t > startDelay:
                print(f"\n=== Loop {loop_count} Performance (Rate: {controllerUpdateRate}Hz, Time: {t:.1f}s) ===")
                print(f"Total loop time:     {total_loop_time:.3f}ms (Target: {target_loop_time:.1f}ms)")
                print(f"  Sensor read:       {sensor_read_time:.3f}ms")
                
                if enableSteeringControl:
                    print(f"  GPS read:          {gps_time:.3f}ms")
                    print(f"  EKF update:        {ekf_time:.3f}ms")
                    if t >= startDelay:
                        print(f"  Speed control:     {speed_control_time:.3f}ms")
                        print(f"  Steering control:  {steering_control_time:.3f}ms")
                    print(f"  Vehicle write:     {write_time:.3f}ms")
                    print(f"  Scope update:      {scope_time:.3f}ms")
                    
                    # Enhanced GPS performance analysis with statistics
                    if enableGPSTimingAnalysis and len(gps_times) > 10:
                        avg_gps_time = statistics.mean(gps_times)
                        max_gps_time = max(gps_times)
                        min_gps_time = min(gps_times)
                        std_gps_time = statistics.stdev(gps_times) if len(gps_times) > 1 else 0
                        
                        # Categorize GPS performance
                        fast_calls = sum(1 for t in gps_times if t < 2.0)
                        moderate_calls = sum(1 for t in gps_times if 2.0 <= t < 5.0)
                        slow_calls = sum(1 for t in gps_times if 5.0 <= t < 10.0)
                        very_slow_calls = sum(1 for t in gps_times if t >= 10.0)
                        
                        print(f"  GPS Analysis (last {len(gps_times)} calls):")
                        print(f"    Average: {avg_gps_time:.3f}ms, Min: {min_gps_time:.3f}ms, Max: {max_gps_time:.3f}ms")
                        print(f"    Std Dev: {std_gps_time:.3f}ms")
                        print(f"    Distribution: Fast(<2ms)={fast_calls}, Moderate(2-5ms)={moderate_calls}, Slow(5-10ms)={slow_calls}, VeryShow(≥10ms)={very_slow_calls}")
                        print(f"    GPS uses {avg_gps_time/target_loop_time*100:.1f}% of loop time")
                        
                        # GPS availability information
                        total_gps_attempts = gps_available_count + gps_unavailable_count
                        gps_availability_rate = gps_available_count / total_gps_attempts * 100 if total_gps_attempts > 0 else 0
                        print(f"    GPS Availability: {gps_available_count}/{total_gps_attempts} ({gps_availability_rate:.1f}%)")
                        
                        if avg_gps_time > 8.0:
                            print(f"    🔴 GPS BOTTLENECK: Average {avg_gps_time:.1f}ms is very slow")
                        elif avg_gps_time > 5.0:
                            print(f"    🟡 GPS MODERATE: Average {avg_gps_time:.1f}ms is somewhat slow")
                        else:
                            print(f"    ✅ GPS GOOD: Average {avg_gps_time:.1f}ms is acceptable")
                            
                else:
                    if t >= startDelay:
                        print(f"  Speed control:     {speed_control_time:.3f}ms")
                    print(f"  Vehicle write:     {write_time:.3f}ms")
                    print(f"  Scope update:      {scope_time:.3f}ms")
                
                # Overall performance analysis with statistics
                if len(all_loop_times) > 10:
                    avg_loop_time = statistics.mean(all_loop_times)
                    std_loop_time = statistics.stdev(all_loop_times) if len(all_loop_times) > 1 else 0
                    
                    print(f"  Recent Loop Performance (last {len(all_loop_times)} loops):")
                    print(f"    Average: {avg_loop_time:.3f}ms, Std Dev: {std_loop_time:.3f}ms")
                    print(f"    Actual rate: {1000.0/avg_loop_time:.1f}Hz (Target: {controllerUpdateRate}Hz)")
                
                # Performance analysis
                if total_loop_time > target_loop_time * 1.5:
                    print(f"  ⚠️  WARNING: Loop time {total_loop_time:.1f}ms exceeds target by {((total_loop_time/target_loop_time-1)*100):.1f}%")
                elif total_loop_time > target_loop_time * 1.2:
                    print(f"  ⚠️  MODERATE: Loop time {total_loop_time:.1f}ms is {((total_loop_time/target_loop_time-1)*100):.1f}% over target")
                else:
                    print(f"  ✅ GOOD: Loop time within target")
            
            # Position logging periodically
            if enablePositionLogging and enableSteeringControl and loop_count % positionLogInterval == 0 and t > startDelay:
                if initial_position_logged and len(position_history) > 0:
                    current_ekf_pos = position_history[-1]  # [x, y, th, t]
                    current_gps_pos = gps_position_history[-1] if len(gps_position_history) > 0 else None
                    
                    print(f"\n--- Position Log (Loop {loop_count}, t={t:.1f}s) ---")
                    print(f"Current EKF Position: ({current_ekf_pos[0]:.6f}, {current_ekf_pos[1]:.6f}, θ={current_ekf_pos[2]:.6f})")
                    if current_gps_pos:
                        print(f"Current GPS Position: ({current_gps_pos[0]:.6f}, {current_gps_pos[1]:.6f}, θ={current_gps_pos[2]:.6f})")
                        # Calculate drift from initial position
                        ekf_drift = np.sqrt((current_ekf_pos[0] - initial_ekf_position[0])**2 + 
                                          (current_ekf_pos[1] - initial_ekf_position[1])**2)
                        gps_drift = np.sqrt((current_gps_pos[0] - initial_gps_position[0])**2 + 
                                          (current_gps_pos[1] - initial_gps_position[1])**2)
                        print(f"Drift from initial: EKF={ekf_drift:.6f}m, GPS={gps_drift:.6f}m")
            
            # Brief warning for very slow loops
            elif enablePerformanceLogging and total_loop_time > target_loop_time * 2.0:
                print(f"Loop {loop_count}: VERY SLOW - {total_loop_time:.1f}ms (Target: {target_loop_time:.1f}ms)")
            #endregion
            
            continue
        # out of while loop : Stop the car
        qcar.read_write_std(throttle= 0, steering= 0)
        
        # Comprehensive Performance Summary (similar to test_pal_gps_performance.py)
        if enableFinalPerformanceLogging:
            print(f"\n{'='*60}")
            print(f"=== COMPREHENSIVE PERFORMANCE SUMMARY ===")
            print(f"{'='*60}")
            
            actual_duration = t
            print(f"Simulation Duration: {actual_duration:.1f}s")
            print(f"Total control loops executed: {loop_count}")
            print(f"Average loop rate: {loop_count/actual_duration:.1f}Hz (Target: {controllerUpdateRate}Hz)")
            
            # Overall loop performance analysis
            if len(all_loop_times) > 0:
                avg_loop = statistics.mean(all_loop_times)
                min_loop = min(all_loop_times)
                max_loop = max(all_loop_times)
                std_loop = statistics.stdev(all_loop_times) if len(all_loop_times) > 1 else 0
                target_loop_time = 1000.0 / controllerUpdateRate
                
                print(f"\nLoop Performance ({len(all_loop_times)} samples):")
                print(f"  Average loop time: {avg_loop:.3f}ms (Target: {target_loop_time:.1f}ms)")
                print(f"  Min loop time: {min_loop:.3f}ms")
                print(f"  Max loop time: {max_loop:.3f}ms")
                print(f"  Std deviation: {std_loop:.3f}ms")
                print(f"  Actual rate achieved: {1000.0/avg_loop:.1f}Hz")
                
                # Loop performance categorization
                fast_loops = sum(1 for t in all_loop_times if t < target_loop_time)
                moderate_loops = sum(1 for t in all_loop_times if target_loop_time <= t < target_loop_time * 1.5)
                slow_loops = sum(1 for t in all_loop_times if t >= target_loop_time * 1.5)
                
                print(f"  Performance Distribution:")
                print(f"    Fast (< target): {fast_loops} ({fast_loops/len(all_loop_times)*100:.1f}%)")
                print(f"    Moderate (target to 1.5x): {moderate_loops} ({moderate_loops/len(all_loop_times)*100:.1f}%)")
                print(f"    Slow (> 1.5x target): {slow_loops} ({slow_loops/len(all_loop_times)*100:.1f}%)")
            
            # GPS Performance Analysis (enhanced like test_pal_gps_performance.py)
            if enableSteeringControl and enableGPSTimingAnalysis:
                total_gps_attempts = gps_available_count + gps_unavailable_count
                gps_availability_percentage = gps_available_count / total_gps_attempts * 100 if total_gps_attempts > 0 else 0
                
                print(f"\nGPS Availability Analysis:")
                print(f"  Total GPS read attempts: {total_gps_attempts}")
                print(f"  GPS data available: {gps_available_count} times")
                print(f"  GPS data unavailable: {gps_unavailable_count} times")
                print(f"  GPS availability rate: {gps_availability_percentage:.1f}%")
                
                # Calculate true GPS rate based on actual availability
                if first_gps_time is not None and last_gps_time is not None and gps_available_count > 1:
                    gps_duration = last_gps_time - first_gps_time
                    true_gps_rate = (gps_available_count - 1) / gps_duration if gps_duration > 0 else 0
                    print(f"  GPS active duration: {gps_duration:.1f}s (from {first_gps_time:.1f}s to {last_gps_time:.1f}s)")
                    print(f"  True GPS rate: {true_gps_rate:.1f}Hz (actual GPS updates per second)")
                    print(f"  Expected GPS rate: {controllerUpdateRate}Hz (controller rate)")
                    print(f"  GPS efficiency: {true_gps_rate/controllerUpdateRate*100:.1f}% of controller rate")
                    
                    # Analysis of GPS rate vs expectation
                    if true_gps_rate < controllerUpdateRate * 0.5:
                        print(f"  🔴 GPS UNDERPERFORMING: True rate {true_gps_rate:.1f}Hz much lower than expected {controllerUpdateRate}Hz")
                    elif true_gps_rate < controllerUpdateRate * 0.8:
                        print(f"  🟡 GPS MODERATE: True rate {true_gps_rate:.1f}Hz somewhat lower than expected {controllerUpdateRate}Hz")
                    else:
                        print(f"  ✅ GPS GOOD: True rate {true_gps_rate:.1f}Hz close to expected {controllerUpdateRate}Hz")
                
                # GPS timing performance (only for available GPS data)
                if len(gps_times) > 0:
                    avg_gps = statistics.mean(gps_times)
                    min_gps = min(gps_times)
                    max_gps = max(gps_times)
                    std_gps = statistics.stdev(gps_times) if len(gps_times) > 1 else 0
                    
                    print(f"\nGPS Timing Performance ({len(gps_times)} valid samples):")
                    print(f"  Average GPS read time: {avg_gps:.3f}ms")
                    print(f"  Min GPS read time: {min_gps:.3f}ms")  
                    print(f"  Max GPS read time: {max_gps:.3f}ms")
                    print(f"  Std deviation: {std_gps:.3f}ms")
                    
                    # GPS call distribution (like test_pal_gps_performance.py)
                    fast_gps = sum(1 for t in gps_times if t < 2.0)
                    moderate_gps = sum(1 for t in gps_times if 2.0 <= t < 5.0)
                    slow_gps = sum(1 for t in gps_times if 5.0 <= t < 10.0)
                    very_slow_gps = sum(1 for t in gps_times if t >= 10.0)
                    
                    print(f"  GPS Call Distribution (when available):")
                    print(f"    Fast (<2ms): {fast_gps} ({fast_gps/len(gps_times)*100:.1f}%)")
                    print(f"    Moderate (2-5ms): {moderate_gps} ({moderate_gps/len(gps_times)*100:.1f}%)")
                    print(f"    Slow (5-10ms): {slow_gps} ({slow_gps/len(gps_times)*100:.1f}%)")
                    print(f"    Very slow (≥10ms): {very_slow_gps} ({very_slow_gps/len(gps_times)*100:.1f}%)")
                    
                    # GPS impact analysis
                    if len(all_loop_times) > 0:
                        avg_loop_time = statistics.mean(all_loop_times)
                        gps_percentage_when_available = (avg_gps / avg_loop_time) * 100
                        gps_percentage_overall = (avg_gps * gps_availability_percentage / 100) / avg_loop_time * 100
                        
                        print(f"  GPS Impact on Loop Performance:")
                        print(f"    When GPS available: uses {gps_percentage_when_available:.1f}% of loop time")
                        print(f"    Overall impact: uses {gps_percentage_overall:.1f}% of total simulation time")
                        print(f"    Time remaining for other operations: {avg_loop_time - avg_gps:.1f}ms (when GPS active)")
                    
                    # GPS Performance conclusions (like test_pal_gps_performance.py)
                    if avg_gps > 8.0:
                        print(f"  🔴 CRITICAL: GPS timing is slow (avg {avg_gps:.1f}ms) when available")
                        print(f"     → GPS read performance needs optimization")
                        print(f"     → Current timing uses {avg_gps/(1000/controllerUpdateRate)*100:.1f}% of target loop time")
                    elif avg_gps > 5.0:
                        print(f"  🟡 WARNING: GPS timing is moderate (avg {avg_gps:.1f}ms) when available")
                        print(f"     → GPS read performance could be improved")
                    else:
                        print(f"  ✅ GOOD: GPS timing performance is acceptable (avg {avg_gps:.1f}ms) when available")
                else:
                    print(f"\nGPS Timing Performance: No valid GPS samples collected")
                    print(f"  🔴 CRITICAL: GPS never provided valid data during simulation")
            
            # Position and Movement Analysis
            if enablePositionLogging and enableSteeringControl and len(position_history) > 0:
                print(f"\nPosition and Movement Analysis:")
                if initial_position_logged:
                    final_ekf_pos = position_history[-1]
                    final_gps_pos = gps_position_history[-1] if len(gps_position_history) > 0 else None
                    
                    print(f"  Initial Positions:")
                    print(f"    Expected: {initialPose}")
                    print(f"    EKF: ({initial_ekf_position[0]:.6f}, {initial_ekf_position[1]:.6f}, {initial_ekf_position[2]:.6f})")
                    print(f"    GPS: ({initial_gps_position[0]:.6f}, {initial_gps_position[1]:.6f}, {initial_gps_position[2]:.6f})")
                    
                    print(f"  Final Positions:")
                    print(f"    EKF: ({final_ekf_pos[0]:.6f}, {final_ekf_pos[1]:.6f}, {final_ekf_pos[2]:.6f})")
                    if final_gps_pos:
                        print(f"    GPS: ({final_gps_pos[0]:.6f}, {final_gps_pos[1]:.6f}, {final_gps_pos[2]:.6f})")
                    
                    # Calculate total movement
                    ekf_total_movement = np.sqrt((final_ekf_pos[0] - initial_ekf_position[0])**2 + 
                                               (final_ekf_pos[1] - initial_ekf_position[1])**2)
                    if final_gps_pos:
                        gps_total_movement = np.sqrt((final_gps_pos[0] - initial_gps_position[0])**2 + 
                                                   (final_gps_pos[1] - initial_gps_position[1])**2)
                        print(f"  Total Movement: EKF={ekf_total_movement:.6f}m, GPS={gps_total_movement:.6f}m")
                    else:
                        print(f"  Total Movement: EKF={ekf_total_movement:.6f}m")
                
                print(f"  Position data points collected: EKF={len(position_history)}, GPS={len(gps_position_history)}")
                print(f"  GPS data collection efficiency: {len(gps_position_history)}/{len(position_history)*1.0:.1%} (GPS data points / total loops)")
                
                # Analysis of GPS data collection pattern
                if len(gps_availability_history) > 10:
                    # Calculate periods where GPS was consistently available/unavailable
                    consecutive_available = 0
                    consecutive_unavailable = 0
                    max_consecutive_available = 0
                    max_consecutive_unavailable = 0
                    
                    for i, (t, available) in enumerate(gps_availability_history):
                        if available:
                            consecutive_available += 1
                            consecutive_unavailable = 0
                            max_consecutive_available = max(max_consecutive_available, consecutive_available)
                        else:
                            consecutive_unavailable += 1
                            consecutive_available = 0
                            max_consecutive_unavailable = max(max_consecutive_unavailable, consecutive_unavailable)
                    
                    print(f"  GPS availability patterns:")
                    print(f"    Max consecutive GPS available: {max_consecutive_available} loops")
                    print(f"    Max consecutive GPS unavailable: {max_consecutive_unavailable} loops")
                    
                    if max_consecutive_unavailable > 50:
                        print(f"    🔴 WARNING: Long periods without GPS data (up to {max_consecutive_unavailable} loops)")
                    elif max_consecutive_unavailable > 20:
                        print(f"    🟡 NOTICE: Some periods without GPS data (up to {max_consecutive_unavailable} loops)")
                    else:
                        print(f"    ✅ GOOD: GPS availability is consistent")
            
            # Component timing breakdown
            if len(all_sensor_times) > 0 and len(all_control_times) > 0:
                avg_sensor = statistics.mean(all_sensor_times)
                avg_control = statistics.mean(all_control_times)
                
                print(f"\nComponent Timing Breakdown:")
                print(f"  Average sensor read time: {avg_sensor:.3f}ms")
                print(f"  Average control time: {avg_control:.3f}ms")
                if enableSteeringControl and len(gps_times) > 0:
                    avg_gps = statistics.mean(gps_times)
                    print(f"  Average GPS time: {avg_gps:.3f}ms")
                    print(f"  Other operations: {statistics.mean(all_loop_times) - avg_sensor - avg_control - avg_gps:.3f}ms")
            
            # Performance recommendations
            print(f"\nPerformance Recommendations:")
            if enableSteeringControl:
                total_gps_attempts = gps_available_count + gps_unavailable_count
                gps_availability_rate = gps_available_count / total_gps_attempts * 100 if total_gps_attempts > 0 else 0
                
                if len(gps_times) > 0:
                    avg_gps = statistics.mean(gps_times)
                    
                    # GPS availability recommendations
                    if gps_availability_rate < 50:
                        print("  🔴 CRITICAL GPS AVAILABILITY: GPS data available less than 50% of the time")
                        print("     → Check GPS initialization and calibration")
                        print("     → Verify GPS sensor configuration")
                    elif gps_availability_rate < 80:
                        print("  🟡 LOW GPS AVAILABILITY: GPS data available less than 80% of the time")
                        print("     → Monitor GPS sensor performance")
                        print("     → Consider GPS configuration optimization")
                    else:
                        print("  ✅ GOOD GPS AVAILABILITY: GPS data consistently available")
                    
                    # GPS timing recommendations
                    if avg_gps > 8.0:
                        print("  🔴 URGENT GPS TIMING: Reduce GPS update rate significantly (to 25-50Hz)")
                        print("  🔴                    GPS timing bottleneck severely impacts real-time performance")
                    elif avg_gps > 5.0:
                        print("  🟡 CONSIDER GPS TIMING: Reducing GPS update rate moderately (to 50-75Hz)")
                    else:
                        print("  ✅ MAINTAIN GPS TIMING: Current GPS timing performance is acceptable")
                        
                    # Calculate effective GPS rate recommendation
                    if first_gps_time is not None and last_gps_time is not None and gps_available_count > 1:
                        gps_duration = last_gps_time - first_gps_time
                        true_gps_rate = (gps_available_count - 1) / gps_duration if gps_duration > 0 else 0
                        print(f"  📊 MEASURED GPS RATE: {true_gps_rate:.1f}Hz actual vs {controllerUpdateRate}Hz expected")
                        
                        if true_gps_rate < 25:
                            print("     → GPS rate is very low, check for systematic issues")
                        elif true_gps_rate < 50:
                            print("     → GPS rate is low but may be sufficient for basic control")
                        elif true_gps_rate < 100:
                            print("     → GPS rate is moderate, good for most applications")
                        else:
                            print("     → GPS rate is high, excellent for precision control")
                else:
                    print("  🔴 CRITICAL: No valid GPS data collected during simulation")
                    print("     → Check GPS sensor connection and initialization")
                    print("     → Verify calibration pose and setup")
            
            if len(all_loop_times) > 0:
                avg_loop = statistics.mean(all_loop_times)
                target_loop = 1000.0 / controllerUpdateRate
                if avg_loop > target_loop * 1.5:
                    print("  ⚠️  OPTIMIZE: Loop performance significantly below target")
                    print("      Consider reducing update rate or optimizing algorithms")
                elif avg_loop > target_loop * 1.1:
                    print("  ⚠️  MONITOR: Loop performance slightly below target")
                else:
                    print("  ✅ EXCELLENT: Loop performance meets target requirements")
            
            print(f"\n{'='*60}")
            print("Performance analysis complete.")
            print(f"{'='*60}\n")

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

    input('Experiment complete. Press any key to exit...')
    #endregion
#endregion