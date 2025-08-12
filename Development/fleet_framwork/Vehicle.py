import threading
import time
import math
import logging
from typing import Any, Optional

from qvl.qcar2 import QLabsQCar2
from src.Controller.CACC import CACC
from src.Controller.idm_control import IDMControl
from src.Controller.DummyController import DummyController, DummyVehicle
from pal.utilities.math import wrap_to_pi

# Import the new lightweight leader and follower controllers
from VehicleLeaderController import VehicleLeaderController
from VehicleFollowerController import VehicleFollowerController


class Vehicle:
    """
    Vehicle class that represents a single vehicle in the fleet.
    Each vehicle runs in its own thread and manages its own control logic.
    """
    
    def __init__(self, vehicle_id: int, qcar: QLabsQCar2, controller_type: str = "CACC", 
                 is_leader: bool = False, config=None, fleet_lock=None):
        """
        Initialize a vehicle instance.
        
        Args:
            vehicle_id: Unique identifier for this vehicle
            qcar: QLabsQCar2 instance for this vehicle
            controller_type: Type of controller ("CACC" or "IDM")
            is_leader: Whether this vehicle is the leader
            config: Configuration object containing vehicle parameters
            fleet_lock: Shared lock for thread synchronization
        """
        self.vehicle_id = vehicle_id
        self.qcar = qcar
        self.controller_type = controller_type
        self.is_leader = is_leader
        self.config = config
        self.fleet_lock = fleet_lock or threading.Lock()
        
        # Thread management
        self.running = False
        self.control_thread = None
        self.update_rate = 100  # Hz
        
        # Vehicle state
        self.current_pos = [0, 0, 0]
        self.current_rot = [0, 0, 0]
        self.velocity = 0.0
        self.prev_pos = None
        self.prev_time = None
        
        # Control parameters
        self.max_steering = config.max_steering if config else 0.6
        self.lookahead_distance = config.lookahead_distance if config else 7.0
        self.k_steering = 2.0
        
        # Leader tracking for followers
        self.leader_vehicle = None
        self.leader_state = None

        # Logging
        self.logger = logging.getLogger(f"Vehicle_{vehicle_id}")
        
        # Control components based on vehicle role
        if self.is_leader:
            self.leader_controller = VehicleLeaderController(
                vehicle_id=self.vehicle_id,
                config=self.config,
                logger=self.logger
            )
            self.follower_controller = None
        else:
            self.leader_controller = None
            self.follower_controller = VehicleFollowerController(
                vehicle_id=self.vehicle_id,
                controller_type=self.controller_type,
                config=self.config,
                logger=self.logger
            )
        

    def set_leader(self, leader_vehicle: 'Vehicle'):
        """Set the leader vehicle for this follower."""
        if not self.is_leader:
            self.leader_vehicle = leader_vehicle
            # Set leader for the follower controller
            if self.follower_controller is not None:
                self.follower_controller.set_leader(leader_vehicle)
            self.logger.info(f"Vehicle {self.vehicle_id} following Vehicle {leader_vehicle.vehicle_id}")
    
    def update_state(self):
        """Update the vehicle's current state from QLabs."""
        try:
            with self.fleet_lock:
                _, pos, rot, _ = self.qcar.get_world_transform()
                self.current_pos = pos
                self.current_rot = rot
                
                # Calculate velocity
                current_time = time.time()
                if self.prev_pos is not None and self.prev_time is not None:
                    dx = pos[0] - self.prev_pos[0]
                    dy = pos[1] - self.prev_pos[1]
                    distance = math.sqrt(dx**2 + dy**2)
                    dt = current_time - self.prev_time
                    self.velocity = distance / dt if dt > 1e-6 else 0.0
                else:
                    self.velocity = 0.0
                    
                self.prev_pos = pos
                self.prev_time = current_time
                
        except Exception as e:
            self.logger.error(f"Error updating state: {e}")
    
    def leader_control_logic(self):
        """Simple leader control logic that delegates to VehicleLeaderController."""
        if not self.is_leader or self.leader_controller is None:
            return
            
        try:
            # Compute control commands using the dedicated leader controller
            forward_speed, steering_angle = self.leader_controller.compute_control(
                current_pos=self.current_pos,
                current_rot=self.current_rot,
                velocity=self.velocity,
                dt=1.0 / self.update_rate
            )
            
            # # Apply commands to QLabs vehicle
            # with self.fleet_lock:
            #     self.qcar.set_velocity_and_request_state(
            #         forward=forward_speed,
            #         turn=steering_angle,
            #         headlights=False,
            #         leftTurnSignal=False,
            #         rightTurnSignal=False,
            #         brakeSignal=False,
            #         reverseSignal=False
            #     )
                
        except Exception as e:
            self.logger.error(f"Leader control error: {e}")
            # # Fallback to simple control
            # try:
            #     with self.fleet_lock:
            #         self.qcar.set_velocity_and_request_state(
            #             forward=0.3, turn=0.0, headlights=False,
            #             leftTurnSignal=False, rightTurnSignal=False,
            #             brakeSignal=False, reverseSignal=False
            #         )
            # except Exception as fallback_error:
            #     self.logger.error(f"Fallback control error: {fallback_error}")
    
    def follower_control_logic(self):
        """Lightweight follower control logic that delegates to VehicleFollowerController."""
        if not self.is_leader and self.follower_controller is not None:
            try:
                # Compute control commands using the dedicated follower controller
                forward_speed, steering_angle = self.follower_controller.compute_control(
                    current_pos=self.current_pos,
                    current_rot=self.current_rot,
                    velocity=self.velocity,
                    dt=1.0 / self.update_rate
                )
                
                # Apply commands to QLabs vehicle
                with self.fleet_lock:
                    self.qcar.set_velocity_and_request_state(
                        forward=forward_speed,
                        turn=steering_angle,
                        headlights=False,
                        leftTurnSignal=False,
                        rightTurnSignal=False,
                        brakeSignal=False,
                        reverseSignal=False
                    )
                    
            except Exception as e:
                self.logger.error(f"Follower control error: {e}")
                # Fallback to stop
                try:
                    with self.fleet_lock:
                        self.qcar.set_velocity_and_request_state(
                            forward=0.0, turn=0.0, headlights=False,
                            leftTurnSignal=False, rightTurnSignal=False,
                            brakeSignal=False, reverseSignal=False
                        )
                except Exception as fallback_error:
                    self.logger.error(f"Fallback control error: {fallback_error}")
        else:
            self.logger.warning("Follower controller not available or vehicle is leader")
    
    def control_loop(self):
        """Main control loop running in its own thread."""
        self.logger.info(f"Vehicle {self.vehicle_id} control loop started (Leader: {self.is_leader})")
        
        while self.running:
            start_time = time.time()
            
            try:
                # Update vehicle state
                self.update_state()
                
                # Execute control logic based on role
                if self.is_leader:
                    self.leader_control_logic()
                else:
                    self.follower_control_logic()
                
                # Maintain update rate
                elapsed = time.time() - start_time
                sleep_time = max(0, (1.0 / self.update_rate) - elapsed)
                print(f"Vehicle {self.vehicle_id} control loop iteration took {elapsed:.4f}s, sleeping for {sleep_time:.4f}s")
                time.sleep(sleep_time)
                
            except Exception as e:
                self.logger.error(f"Control loop error: {e}")
                time.sleep(0.1)  # Brief pause before retry
    
    def start(self):
        """Start the vehicle's control thread."""
        if self.control_thread is None or not self.control_thread.is_alive():
            self.running = True
            self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
            self.control_thread.start()
            self.logger.info(f"Vehicle {self.vehicle_id} started")
    
    def stop(self):
        """Stop the vehicle's control thread."""
        self.running = False
        if self.control_thread is not None:
            self.control_thread.join(timeout=2.0)
        
        # Stop the vehicle
        try:
            with self.fleet_lock:
                self.qcar.set_velocity_and_request_state(
                    forward=0.0, turn=0.0, headlights=False,
                    leftTurnSignal=False, rightTurnSignal=False,
                    brakeSignal=False, reverseSignal=False
                )
        except Exception as e:
            self.logger.error(f"Error stopping vehicle: {e}")
        
        # Clean up controllers
        if self.is_leader and self.leader_controller is not None:
            try:
                self.leader_controller.stop_control()
            except Exception as e:
                self.logger.error(f"Error stopping leader controller: {e}")
        
        if not self.is_leader and self.follower_controller is not None:
            try:
                self.follower_controller.stop_control()
            except Exception as e:
                self.logger.error(f"Error stopping follower controller: {e}")
            
        self.logger.info(f"Vehicle {self.vehicle_id} stopped")
    
    def is_alive(self):
        """Check if the vehicle's control thread is alive."""
        return self.control_thread is not None and self.control_thread.is_alive()
    
    def join(self, timeout=None):
        """Wait for the vehicle's control thread to finish."""
        if self.control_thread is not None:
            self.control_thread.join(timeout)
    
    def get_state(self):
        """Get current vehicle state."""
        state = {
            'vehicle_id': self.vehicle_id,
            'position': self.current_pos.copy(),
            'rotation': self.current_rot.copy(),
            'velocity': self.velocity,
            'is_leader': self.is_leader,
            'running': self.running
        }
        
        # # Add role-specific state information
        # if self.is_leader and self.leader_controller is not None:
        #     leader_state = self.leader_controller.get_control_state()
        #     state.update({
        #         'leader_elapsed_time': leader_state['elapsed_time'],
        #         'throttle_command': leader_state['throttle_command'],
        #         'steering_command': leader_state['steering_command'],
        #         'reference_velocity': leader_state['reference_velocity'],
        #         'waypoint_count': leader_state['waypoint_count'],
        #         'leader_initialized': leader_state['initialized']
        #     })
        # elif not self.is_leader and self.follower_controller is not None:
        #     follower_state = self.follower_controller.get_control_state()
        #     state.update({
        #         'controller_type': follower_state['controller_type'],
        #         'follower_initialized': follower_state['initialized'],
        #         'has_leader': follower_state['has_leader'],
        #         'leader_id': follower_state['leader_id'],
        #         'lookahead_distance': follower_state['lookahead_distance'],
        #         'max_steering': follower_state['max_steering'],
        #         'k_steering': follower_state['k_steering']
        #     })
        
        return state
