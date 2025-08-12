import time
import numpy as np
import logging
from typing import Any, Optional

from pal.products.qcar import QCar, QCarGPS
from hal.content.qcar_functions import QCarEKF
from src.OpenRoad import OpenRoad
from hal.products.mats import SDCSRoadMap
from src.Controller.ControllerLeader import SpeedController, SteeringController


class VehicleLeaderController:
    """
    Dedicated controller for leader vehicles that encapsulates all the complex
    leader control logic from ControlLeader class.
    """
    
    def __init__(self, vehicle_id: int, config=None, logger=None):
        """
        Initialize the leader controller.
        
        Args:
            vehicle_id: ID of the vehicle this controller manages
            config: Configuration object containing parameters
            logger: Logger instance for this controller
        """
        self.vehicle_id = vehicle_id
        self.config = config
        self.logger = logger or logging.getLogger(f"LeaderController_{vehicle_id}")
        
        # Control parameters (from ControlLeader)
        self.controllerUpdateRate = 50
        self.K_p = 0.1
        self.K_i = 0.8
        self.enableSteeringControl = True
        self.K_stanley = 0.8
        self.calibrationPose = [0, 2, -np.pi/3]
        self.calibrate = True
        self.startDelay = 0.5
        
        # Control state
        self.start_time = None
        self.delta = 0  # Steering command
        self.u = 0      # Throttle command
        self.max_steering = config.max_steering if config else 0.6
        
        # Path and controllers
        self.waypointSequence = None
        self.InitialPose = None
        self.speedController = None
        self.steeringController = None
        self.ekf = None
        self.gps = None

        self.qcar = QCar(readMode=1, frequency=self.controllerUpdateRate)

        
        # Initialize components
        self._init_path_and_controllers()
        
        self.logger.info(f"Leader controller initialized for vehicle {vehicle_id}")
    
    def _init_path_and_controllers(self):
        """Initialize path generation and controllers."""
        # Get configuration parameters
        road_type = self.config.get_road_type_name() if self.config else "OpenRoad"
        node_sequence = self.config.get_node_sequence() if self.config else [0, 1]
        
        # Generate path
        self.waypointSequence, self.InitialPose = self._generate_path(road_type, node_sequence)
        
        # Initialize speed controller
        self.speedController = SpeedController(
            kp=self.K_p,
            ki=self.K_i
        )
        
        # Initialize steering controller
        if self.enableSteeringControl:
            self.steeringController = SteeringController(
                waypoints=self.waypointSequence,
                k=self.K_stanley
            )
        
        self.logger.info(f"Generated path with {len(self.waypointSequence)} waypoints")
    
    def _generate_path(self, road_type: str, node_sequence: list):
        """Generate path for leader vehicle (from ControlLeader.PathGeneration)."""
        if road_type == "OpenRoad":
            roadmap = OpenRoad()
        elif road_type == "Studio":
            roadmap = SDCSRoadMap()
        else:
            raise ValueError(f"Unknown road type: {road_type}")
        
        waypointSequence = roadmap.generate_path(node_sequence)
        InitialPose = roadmap.get_node_pose(node_sequence[0]).squeeze()
        
        return waypointSequence, InitialPose
    
    def _get_vref(self, t):
        """Get reference velocity based on time (from ControlLeader.vref)."""
        road_type = self.config.get_road_type_name() if self.config else "OpenRoad"
        
        if road_type == "OpenRoad":
            if t < 5:
                v_ref = 2
            elif t < 10:
                v_ref = 0
            elif t < 15:
                v_ref = -0.5
            elif t < 20:
                v_ref = 1
            else:
                v_ref = 2
            return v_ref
        elif road_type == "Studio":
            return 0.3
        else:
            return 0.3
    
    def start_control(self,dt = 0.1):
        """Start the control session (call this when vehicle starts)."""
        if self.start_time is None:
            self.start_time = time.time()
            # self.qcar = QCar(readMode=1, frequency=dt)

            
            # Initialize EKF for state estimation
            if self.enableSteeringControl or self.calibrate:
                self.ekf = QCarEKF(x_0=self.InitialPose)
                self.gps = QCarGPS(initialPose=self.calibrationPose, calibrate=self.calibrate)

            
            self.logger.info("Leader control session started")
    
    def compute_control(self, current_pos, current_rot, velocity, dt=0.1):
        """
        Compute control commands based on current vehicle state.
        This is the main control logic from ControlLeader.run().
        
        Args:
            current_pos: Current position [x, y, z] from QLabs
            current_rot: Current rotation [roll, pitch, yaw] from QLabs  
            velocity: Current velocity from vehicle
            dt: Time step for control update
            
        Returns:
            tuple: (forward_speed, steering_angle) control commands
        """
        if self.start_time is None:
            self.start_control(dt)
        
        # Calculate elapsed time
        t = time.time() - self.start_time
        
        # Get reference velocity
        vref = self._get_vref(t)
        
        # # Extract position and orientation
        # x, y = current_pos[0], current_pos[1]
        # th = current_rot[2]  # Heading in radians
        

        self.qcar.read()
        # State estimation with EKF (like in ControlLeader)
        if self.enableSteeringControl and self.ekf is not None:
            if self.gps.readGPS():
                y_gps = np.array([self.gps.position[0],
                                self.gps.position[1],
                                self.gps.orientation[2]])
                self.ekf.update([self.qcar.motorTach, self.delta],
                            dt,
                            y_gps,
                            self.qcar.gyroscope[2],)
            else:
                self.ekf.update([self.qcar.motorTach, self.delta],
                            dt,
                            None,
                            self.qcar.gyroscope[2],)
            x = self.ekf.x_hat[0, 0]
            y = self.ekf.x_hat[1, 0]
            th = self.ekf.x_hat[2, 0]
            p = np.array([x, y]) + np.array([np.cos(th), np.sin(th)]) * 0.2
        
        
        # Current velocity
        # v = velocity
        v = self.qcar.motorTach
        
        # Control logic (from ControlLeader.run())
        if t < self.startDelay:
            self.u = 0
            self.delta = 0
        else:
            # Speed control
            self.u = self.speedController.update(v, vref, dt)
            
            # Steering control
            if self.enableSteeringControl :
                self.delta = self.steeringController.update(p, th, v)
            else:
                self.delta = 0
        
        # Convert and clamp commands
        forward_speed = self.u
        steering_angle = self.delta
        
        # Clamp to safe ranges
        forward_speed = max(-2.0, min(2.0, forward_speed))
        steering_angle = max(-self.max_steering, min(self.max_steering, steering_angle))
        
        self.qcar.write(forward_speed, steering_angle)
        
        return forward_speed, steering_angle
    
    def get_control_state(self):
        """Get current control state information."""
        if self.start_time is None:
            return {
                'initialized': False,
                'elapsed_time': 0,
                'throttle_command': 0,
                'steering_command': 0,
                'reference_velocity': 0,
                'waypoint_count': len(self.waypointSequence) if self.waypointSequence is not None else 0
            }
        
        elapsed_time = time.time() - self.start_time
        return {
            'initialized': True,
            'elapsed_time': elapsed_time,
            'throttle_command': self.u,
            'steering_command': self.delta,
            'reference_velocity': self._get_vref(elapsed_time),
            'waypoint_count': len(self.waypointSequence) if self.waypointSequence is not None else 0
        }
    
    def stop_control(self):
        """Stop the control session and cleanup."""
        if self.ekf is not None:
            # Clean up EKF if needed
            pass
        
        self.start_time = None
        self.qcar.write(0, 0)

        self.u = 0
        self.delta = 0
        
        self.logger.info("Leader control session stopped")
    
    def reset_control(self):
        """Reset the control session (restart timing)."""
        self.stop_control()
        self.start_control()
        self.logger.info("Leader control session reset")
