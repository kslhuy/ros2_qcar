import math
import time
import logging
from typing import Optional, Tuple

from src.Controller.CACC import CACC
from src.Controller.idm_control import IDMControl
from src.Controller.DummyController import DummyController, DummyVehicle
from pal.utilities.math import wrap_to_pi


class VehicleFollowerController:
    """
    Dedicated follower controller class that handles all follower-specific control logic.
    This separates the control logic from the Vehicle class, making it more modular.
    """
    
    def __init__(self, vehicle_id: int, controller_type: str = "CACC", config=None, logger=None):
        """
        Initialize the follower controller.
        
        Args:
            vehicle_id: ID of the follower vehicle
            controller_type: Type of controller ("CACC" or "IDM")
            config: Configuration object containing vehicle parameters
            logger: Logger instance for this controller
        """
        self.vehicle_id = vehicle_id
        self.controller_type = controller_type
        # print(controller_type , "control type")
        self.config = config
        self.logger = logger or logging.getLogger(f"FollowerController_{vehicle_id}")
        
        # Control parameters - use config if available, otherwise defaults
        self.lookahead_distance = config.get('lookahead_distance', 0.4) if config else 0.4
        # print(f"Vehicle {self.vehicle_id} lookahead distance: {self.lookahead_distance}")
        self.max_steering = config.get('max_steering', 0.6) if config else 0.6
        self.k_steering = 2.0  # Steering gain
        
        # State tracking for legacy compatibility (if needed)
        self.leader_state = None
        
        self.logger.info(f"Follower controller initialized for vehicle {vehicle_id} with {controller_type}")
        
        # State tracking
        self.prev_pos = None
        self.prev_time = None
        self.velocity = 0.0
        self.initialized = False
        # Initialize the longitudinal controller
        self._init_controller()
        
    
    def _init_controller(self):
        """Initialize the appropriate longitudinal controller for this vehicle."""
        # print(f"Vehicle {self.vehicle_id}:  {self.config}")
        print(f"Vehicle {self.vehicle_id}: Initializing controller with config type: {type(self.config)}")
        try:
            if self.config is not None:
                # Handle config as dictionary - check if dummy_controller_params is vehicle-specific or global
                dummy_controller_params = self.config.get('dummy_controller_params', {})
                print(f"Vehicle {self.vehicle_id}: dummy_controller_params = {dummy_controller_params}")
                print(f"Vehicle {self.vehicle_id}: dummy_controller_params keys = {list(dummy_controller_params.keys()) if dummy_controller_params else 'None'}")
                
                # Check if it's vehicle-specific (nested dict) or global params
                if str(self.vehicle_id) in dummy_controller_params:
                    # Vehicle-specific params: config['dummy_controller_params']['1']
                    dummy_params = dummy_controller_params[str(self.vehicle_id)]
                    print(f"Vehicle {self.vehicle_id}: Using vehicle-specific params: {dummy_params}")
                elif isinstance(dummy_controller_params, dict) and 'alpha' in dummy_controller_params:
                    # Global params directly in dummy_controller_params
                    dummy_params = dummy_controller_params
                    print(f"Vehicle {self.vehicle_id}: Using global params: {dummy_params}")
                else:
                    # No params found, use defaults
                    dummy_params = None
                    print(f"Vehicle {self.vehicle_id}: No params found, using defaults")
                    
                dummy_controller = DummyController(self.vehicle_id, dummy_params)
            else:
                print(f"Vehicle {self.vehicle_id}: Config is None, using default DummyController")
                dummy_controller = DummyController(self.vehicle_id)
                
            if self.controller_type == "CACC":
                self.controller = CACC(dummy_controller)
            elif self.controller_type == "IDM":
                self.controller = IDMControl(dummy_controller)
            else:
                raise ValueError(f"Unknown controller type: {self.controller_type}")
                
            self.initialized = True
            # self.logger.info(f"Controller {self.controller_type} initialized successfully")
            print(f"Vehicle {self.vehicle_id}: Controller {self.controller_type} initialized successfully")
            
        except Exception as e:
            print(f"Vehicle {self.vehicle_id}: Error initializing controller: {e}")
            # self.logger.error(f"Error initializing controller: {e}")
            self.initialized = False

    def compute_control(self, current_pos: list, current_rot: list, current_velocity: float, 
                       leader_pos: list, leader_rot: list, leader_velocity: float, 
                       leader_timestamp: float, dt: float) -> Tuple[float, float]:
        """
        Compute control commands for the follower vehicle.
        
        Args:
            current_pos: Current position [x, y, z] of the follower
            current_rot: Current rotation [roll, pitch, yaw] of the follower
            current_velocity: Current velocity of the follower
            leader_pos: Leader position [x, y, z]
            leader_rot: Leader rotation [roll, pitch, yaw]
            leader_velocity: Leader velocity
            leader_timestamp: Timestamp of leader data
            dt: Time step
            
        Returns:
            Tuple of (forward_speed, steering_angle)
        """
        if not self.initialized:
            # self.logger.warning("Controller not initialized, returning zero commands")
            print(f"Vehicle {self.vehicle_id}: Controller not initialized")
            return 0.0, 0.0
        
        try:
            # Compute longitudinal control (speed command)
            speed_cmd = self._compute_longitudinal_control(
                current_pos, current_rot, current_velocity,
                leader_pos, leader_rot, leader_velocity
            )
            
            # Compute lateral control (steering command)
            steering_cmd = self._compute_lateral_control(
                current_pos, current_rot,
                leader_pos, leader_rot
            )
            
            # Log control commands for debugging
            self.logger.debug(f"Control commands - Speed: {speed_cmd:.3f}, Steering: {steering_cmd:.3f}")
            
            return speed_cmd, steering_cmd
            
        except Exception as e:
            self.logger.error(f"Error computing control: {e}")
            return 0.0, 0.0
    
    def _compute_longitudinal_control(self, current_pos: list, current_rot: list, velocity: float,
                                    leader_pos: list, leader_rot: list, leader_velocity: float) -> float:
        """
        Compute the longitudinal control (speed command) using the configured controller.
        
        Returns:
            Speed command
        """
        try:
            # Prepare state vectors
            follower_state = [current_pos[0], current_pos[1], current_rot[2], velocity]
            leader_state = [leader_pos[0], leader_pos[1], leader_rot[2], leader_velocity]
            
            # Use direct computation for CACC controller (much cleaner and faster)
            if self.controller_type == "CACC":
                speed_cmd = self.controller.compute_cacc_acceleration(follower_state, leader_state)
            else:
                # For IDM or other controllers, use the legacy interface with DummyVehicle
                speed_cmd = self._compute_legacy_control(follower_state, leader_state)
            
            # Ensure speed command is non-negative
            speed_cmd = max(0, speed_cmd)
            
            return speed_cmd
            
        except Exception as e:
            self.logger.error(f"Error in longitudinal control: {e}")
            return 0.0
    
    def _compute_legacy_control(self, follower_state: list, leader_state: list) -> float:
        """
        Legacy control computation for non-CACC controllers.
        This maintains the old behavior for IDM and other controllers.
        """
        # Create dummy leader vehicle only when needed
        dummy_leader = DummyVehicle(leader_state, vehicle_id=0)
        
        # Use a more efficient approach: store original method once
        if not hasattr(self, '_original_get_surrounding_vehicles'):
            self._original_get_surrounding_vehicles = self.controller.controller.get_surrounding_vehicles
        
        # Temporarily override method
        self.controller.controller.get_surrounding_vehicles = lambda *args, **kwargs: (None, [dummy_leader], None, None)
        
        try:
            # Get optimal control input from the controller
            _, input_u, _ = self.controller.get_optimal_input(
                host_car_id=self.vehicle_id,
                state=follower_state,
                last_input=None,
                lane_id=None,
                input_log=None,
                initial_lane_id=None,
                direction_flag=None,
                type_state="true",
                acc_flag=0
            )
            
            speed_cmd = input_u[0]
            
        finally:
            # Restore original method
            self.controller.controller.get_surrounding_vehicles = self._original_get_surrounding_vehicles
        
        return speed_cmd
    
    def _compute_lateral_control(self, current_pos: list, current_rot: list,
                               leader_pos: list, leader_rot: list) -> float:
        """
        Compute the lateral control (steering command) using pure pursuit.
        
        Returns:
            Steering command
        """
        try:
            # Calculate target position (behind leader with lookahead)
            target_x = leader_pos[0] - self.lookahead_distance * math.cos(leader_rot[2])
            target_y = leader_pos[1] - self.lookahead_distance * math.sin(leader_rot[2])
            
            # Calculate steering command using pure pursuit
            dx = target_x - current_pos[0]
            dy = target_y - current_pos[1]
            target_angle = math.atan2(dy, dx)
            heading_error = wrap_to_pi(target_angle - current_rot[2])
            
            # Apply steering gain and limit
            steering_cmd = -self.k_steering * heading_error
            steering_cmd = max(-self.max_steering, min(self.max_steering, steering_cmd))
            
            return steering_cmd
            
        except Exception as e:
            self.logger.error(f"Error in lateral control: {e}")
            return 0.0
    
    def update_parameters(self, **kwargs):
        """Update control parameters dynamically."""
        if 'lookahead_distance' in kwargs:
            self.lookahead_distance = kwargs['lookahead_distance']
            self.logger.info(f"Updated lookahead distance to {self.lookahead_distance}")
            
        if 'max_steering' in kwargs:
            self.max_steering = kwargs['max_steering']
            self.logger.info(f"Updated max steering to {self.max_steering}")
            
        if 'k_steering' in kwargs:
            self.k_steering = kwargs['k_steering']
            self.logger.info(f"Updated steering gain to {self.k_steering}")
    
    def get_control_state(self) -> dict:
        """Get the current state of the controller."""
        return {
            'vehicle_id': self.vehicle_id,
            'controller_type': self.controller_type,
            'initialized': self.initialized,
            'has_leader': False,  # No longer tracking leader_vehicle object
            'leader_id': None,
            'lookahead_distance': self.lookahead_distance,
            'max_steering': self.max_steering,
            'k_steering': self.k_steering
        }
    
    def stop_control(self):
        """Stop the controller and clean up resources."""
        self.logger.info(f"Stopping follower controller for vehicle {self.vehicle_id}")
        self.initialized = False
