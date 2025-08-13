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
from CommHandler import CommHandler
from StateQueue import StateQueue

from src.GPS_sim.md_gps_sync import GPSSync
from md_logging_config import get_individual_vehicle_logger, get_communication_logger, get_gps_logger, get_control_logger

class Vehicle:
    """
    Vehicle class that represents a single vehicle in the fleet.
    Each vehicle runs in its own thread and manages its own control logic.
    """
    
    def __init__(self, vehicle_id: int, qcar: QLabsQCar2, controller_type: str = "CACC", 
                 is_leader: bool = False, config=None, fleet_lock=None, 
                 target_ip: str = "127.0.0.1", base_send_port: int = 8000, 
                 base_recv_port: int = 9000, base_ack_port: int = 10000,
                 gps_server_ip: str = "127.0.0.1", gps_server_port: int = 8001):
        """
        Initialize a vehicle instance.
        
        Args:
            vehicle_id: Unique identifier for this vehicle
            qcar: QLabsQCar2 instance for this vehicle
            controller_type: Type of controller ("CACC" or "IDM")
            is_leader: Whether this vehicle is the leader
            config: Configuration object containing vehicle parameters
            fleet_lock: Shared lock for thread synchronization
            target_ip: IP address for communication (default: localhost)
            base_send_port: Exact port for sending messages (used as-is)
            base_recv_port: Exact port for receiving messages (used as-is)
            base_ack_port: Exact port for ACK messages (used as-is)
            gps_server_ip: IP address of GPS time synchronization server
            gps_server_port: Port of GPS time synchronization server
        """
        self.vehicle_id = vehicle_id
        self.qcar = qcar
        self.controller_type = controller_type
        self.is_leader = is_leader
        self.config = config
        self.fleet_lock = fleet_lock or threading.Lock()
        
        # Thread management
        self.running = threading.Event()  # Changed to Event for better communication control
        self.control_thread = None
        self.send_thread = None
        self.receive_thread = None
        self.gps_sync_thread = None
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
        
        # Communication setup - use exact ports provided (no addition of vehicle_id)
        self.target_ip = target_ip
        self.send_port = base_send_port
        self.recv_port = base_recv_port
        self.ack_port = base_ack_port

        # Specialized logging setup
        self.logger = get_individual_vehicle_logger(vehicle_id)  # Individual vehicle log file
        self.comm_logger = get_communication_logger(vehicle_id)   # Communication events
        self.gps_logger = get_gps_logger(vehicle_id)             # GPS synchronization
        self.control_logger = get_control_logger(vehicle_id)     # Control operations
        
        # GPS Time Synchronization
        self.gps_sync = GPSSync(gps_server_ip, gps_server_port, vehicle_id)
        self.sync_interval = 5.0  # Sync every 5 seconds
        self.last_sync_attempt = time.time()
        
        # State Queue for managing received states with time validation
        self.state_queue = StateQueue(
            max_queue_size=50,
            max_age_seconds=1.0,
            max_delay_threshold=0.5,
            logger=self.logger
        )
        
        # Initialize communication handler
        self.comm = CommHandler(
            vehicle_id=self.vehicle_id,
            target_ip=self.target_ip,
            send_port=self.send_port,
            recv_port=self.recv_port,
            ack_port=self.ack_port,
            logger=self.comm_logger,  # Use specialized communication logger
            running_flag=self.running,
            vehicle=self
        )
        
        # Control components based on vehicle role
        if self.is_leader:
            self.leader_controller = VehicleLeaderController(
                vehicle_id=self.vehicle_id,
                config=self.config,
                logger=self.control_logger  # Use specialized control logger
            )
            self.follower_controller = None
        else:
            self.leader_controller = None
            self.follower_controller = VehicleFollowerController(
                vehicle_id=self.vehicle_id,
                controller_type=self.controller_type,
                config=self.config,
                logger=self.control_logger  # Use specialized control logger
            )
        

    def set_leader(self, leader_vehicle: 'Vehicle'):
        """Set the leader vehicle for this follower."""
        if not self.is_leader:
            self.leader_vehicle = leader_vehicle
            # Set leader for the follower controller
            if self.follower_controller is not None:
                self.follower_controller.set_leader(leader_vehicle)
            
            # Update communication target to leader's receive port
            self.comm.target_ip = leader_vehicle.target_ip
            self.comm.send_port = leader_vehicle.recv_port
            self.logger.info(f"Vehicle {self.vehicle_id} following Vehicle {leader_vehicle.vehicle_id}")
            self.logger.info(f"Communication target: {self.comm.target_ip}:{self.comm.send_port}")
    
    def set_communication_target(self, target_vehicle: 'Vehicle'):
        """Set communication target to another vehicle."""
        self.comm.target_ip = target_vehicle.target_ip
        self.comm.send_port = target_vehicle.recv_port
        self.logger.info(f"Vehicle {self.vehicle_id} communication target set to Vehicle {target_vehicle.vehicle_id}")
        self.logger.info(f"Communication target: {self.comm.target_ip}:{self.comm.send_port}")
    
    def update_state(self):
        """Update the vehicle's current state from QLabs."""
        try:
            with self.fleet_lock:
                _, pos, rot, _ = self.qcar.get_world_transform()
                self.current_pos = pos
                self.current_rot = rot
                
                # Calculate velocity using GPS-synchronized time
                current_time = self.gps_sync.get_synced_time()
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

    def gps_sync_loop(self):
        """GPS synchronization loop - runs in dedicated thread."""
        self.logger.info(f"Starting GPS sync thread for Vehicle {self.vehicle_id}")
        
        # Initial synchronization
        try:
            self.gps_sync.sync_with_gps()
            self.logger.info(f"Initial GPS sync completed for Vehicle {self.vehicle_id}")
        except Exception as e:
            self.logger.error(f"Initial GPS sync failed: {e}")
        
        while self.running.is_set():
            try:
                current_time = time.time()
                
                # Periodic GPS synchronization
                if current_time - self.last_sync_attempt >= self.sync_interval:
                    self.gps_sync.sync_with_gps()
                    self.last_sync_attempt = current_time
                    self.logger.debug(f"GPS sync completed for Vehicle {self.vehicle_id}")
                
                # Clean up old states in the queue
                self.state_queue.cleanup_old_states(self.gps_sync)
                
                # Sleep until next sync check
                time.sleep(1.0)  # Check every second
                
            except Exception as e:
                self.logger.error(f"GPS sync loop error: {e}")
                time.sleep(1.0)
        
        self.logger.info(f"GPS sync thread exited for Vehicle {self.vehicle_id}")

    def process_received_state(self, received_state: dict):
        """
        Process received state data through StateQueue for validation.
        This method should be called by CommHandler when new states are received.
        
        Args:
            received_state: State data received from another vehicle
        """
        try:
            # Add state to queue with GPS time validation
            if self.state_queue.add_state(received_state, self.gps_sync):
                self.logger.debug(f"Vehicle {self.vehicle_id}: Valid state added to queue from vehicle {received_state.get('id')}")
                
                # Update leader_state for followers using the latest valid state
                if not self.is_leader:
                    # Get the most recent valid state from the leader
                    latest_leader_state = self.state_queue.get_latest_valid_state(
                        sender_id=self.leader_vehicle.vehicle_id if self.leader_vehicle else None
                    )
                    
                    if latest_leader_state:
                        self.leader_state = latest_leader_state
                        self.logger.debug(f"Updated leader_state from validated queue: {latest_leader_state}")
            else:
                self.logger.warning(f"Vehicle {self.vehicle_id}: Invalid state rejected from vehicle {received_state.get('id')}")
                
        except Exception as e:
            self.logger.error(f"Error processing received state: {e}")
    
    def get_interpolated_leader_state(self, target_time: Optional[float] = None) -> Optional[dict]:
        """
        Get interpolated leader state for more accurate control.
        
        Args:
            target_time: Target GPS time for interpolation (default: current time)
            
        Returns:
            Interpolated leader state or None if not available
        """
        if self.is_leader or not self.leader_vehicle:
            return None
            
        if target_time is None:
            target_time = self.gps_sync.get_synced_time()
        
        try:
            return self.state_queue.get_interpolated_state(
                target_time=target_time,
                sender_id=self.leader_vehicle.vehicle_id
            )
        except Exception as e:
            self.logger.error(f"Error getting interpolated leader state: {e}")
            return None
    
    def leader_control_logic(self):
        """Simple leader control logic that delegates to VehicleLeaderController."""
        if not self.is_leader or self.leader_controller is None:
            return
            
        try:
            # Compute control commands using the dedicated leader controller
            # Inside leader controller they handle apply the control commands to Qcar
            forward_speed, steering_angle = self.leader_controller.compute_control(
                current_pos=self.current_pos,
                current_rot=self.current_rot,
                velocity=self.velocity,
                dt=1.0 / self.update_rate
            )
            
                
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
                # Use GPS-synchronized time for better prediction
                current_gps_time = self.gps_sync.get_synced_time()
                
                # Get the best available leader state data from validated queue
                leader_data = None
                
                # Try to get interpolated state for current time (most accurate)
                interpolated_state = self.get_interpolated_leader_state(current_gps_time)
                if interpolated_state:
                    leader_data = {
                        'position': interpolated_state.get('pos', [0, 0, 0]),
                        'rotation': interpolated_state.get('rot', [0, 0, 0]),
                        'velocity': interpolated_state.get('v', 0.0),
                        'timestamp': interpolated_state.get('timestamp', current_gps_time),
                        'interpolated': interpolated_state.get('interpolated', False),
                        'processing_delay': interpolated_state.get('processing_delay', 0.0)
                    }
                    self.logger.debug(f"Using interpolated leader data: pos={leader_data['position']}, "
                                    f"v={leader_data['velocity']:.3f}, interpolated={leader_data['interpolated']}")
                
                # Fallback: use latest valid state if interpolation not available
                elif self.leader_state is not None:
                    # Validate that the leader_state is recent enough
                    state_age = current_gps_time - self.leader_state.get('timestamp', 0)
                    if state_age <= 1.0:  # Use state if less than 1 second old
                        leader_data = {
                            'position': self.leader_state.get('pos', [0, 0, 0]),
                            'rotation': self.leader_state.get('rot', [0, 0, 0]),
                            'velocity': self.leader_state.get('v', 0.0),
                            'timestamp': self.leader_state.get('timestamp', current_gps_time),
                            'interpolated': False,
                            'state_age': state_age
                        }
                        self.logger.debug(f"Using fallback leader data: age={state_age:.3f}s")
                    else:
                        self.logger.warning(f"Leader state too old ({state_age:.3f}s), stopping vehicle")
                        # Stop vehicle if leader data is too old
                        with self.fleet_lock:
                            self.qcar.set_velocity_and_request_state(
                                forward=0.0, turn=0.0, headlights=False,
                                leftTurnSignal=False, rightTurnSignal=False,
                                brakeSignal=False, reverseSignal=False
                            )
                        return
                
                # If no valid leader data available, stop the vehicle
                if leader_data is None:
                    self.logger.warning("No valid leader data available, stopping vehicle")
                    with self.fleet_lock:
                        self.qcar.set_velocity_and_request_state(
                            forward=0.0, turn=0.0, headlights=False,
                            leftTurnSignal=False, rightTurnSignal=False,
                            brakeSignal=False, reverseSignal=False
                        )
                    return

                # Compute control commands using the dedicated follower controller
                forward_speed, steering_angle = self.follower_controller.compute_control(
                    current_pos=self.current_pos,
                    current_rot=self.current_rot,
                    velocity=self.velocity,
                    dt=1.0 / self.update_rate,
                    leader_data=leader_data  # Pass validated leader data to controller
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
        
        while self.running.is_set():
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
                # print(f"Vehicle {self.vehicle_id} control loop iteration took {elapsed:.4f}s, sleeping for {sleep_time:.4f}s")
                time.sleep(sleep_time)
                
            except Exception as e:
                self.logger.error(f"Control loop error: {e}")
                time.sleep(0.1)  # Brief pause before retry
    
    def start(self):
        """Start the vehicle's control thread and communication threads."""
        if self.control_thread is None or not self.control_thread.is_alive():
            self.running.set()  # Set the event flag
            
            # Start GPS synchronization thread first
            self.gps_sync_thread = threading.Thread(target=self.gps_sync_loop, daemon=True)
            self.gps_sync_thread.start()
            self.logger.info(f"Vehicle {self.vehicle_id} GPS sync thread started")
            
            # Small delay to allow initial GPS sync
            time.sleep(0.2)

            # Start communication threads
            self.send_thread = threading.Thread(target=self.comm.send_state, daemon=True)
            self.receive_thread = threading.Thread(target=self.comm.receive_messages, daemon=True)
            self.logger.debug("Starting send and receive threads")
            # Small delay to allow initial GPS sync
            time.sleep(0.2)


            # Start control thread
            self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
            self.control_thread.start()
            

            self.send_thread.start()
            time.sleep(0.1)  # Small delay to stagger thread startup
            self.receive_thread.start()
            
            self.logger.info(f"Vehicle {self.vehicle_id} started with all threads (GPS sync, control, communication)")
    
    def stop(self):
        """Stop the vehicle's control thread and communication threads."""
        self.running.clear()  # Clear the event flag
        
        # Stop GPS sync thread
        if self.gps_sync_thread is not None:
            self.gps_sync_thread.join(timeout=1.0)
        
        # Stop communication threads
        if self.send_thread is not None:
            self.send_thread.join(timeout=1.0)
        if self.receive_thread is not None:
            self.receive_thread.join(timeout=1.0)
            
        # Clean up communication handler
        try:
            self.comm.cleanup()
        except Exception as e:
            self.logger.error(f"Error cleaning up communication handler: {e}")
        
        # Stop control thread
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
        
        # Clear state queue
        self.state_queue.clear()
            
        self.logger.info(f"Vehicle {self.vehicle_id} stopped")
    
    def is_alive(self):
        """Check if the vehicle's control thread and communication threads are alive."""
        control_alive = self.control_thread is not None and self.control_thread.is_alive()
        send_alive = self.send_thread is not None and self.send_thread.is_alive()
        receive_alive = self.receive_thread is not None and self.receive_thread.is_alive()
        
        return control_alive and self.running.is_set()
    
    def join(self, timeout=None):
        """Wait for the vehicle's control thread to finish."""
        if self.control_thread is not None:
            self.control_thread.join(timeout)
    
    def get_state(self):
        """Get current vehicle state with enhanced information."""
        state = {
            'vehicle_id': self.vehicle_id,
            'position': self.current_pos.copy(),
            'rotation': self.current_rot.copy(),
            'velocity': self.velocity,
            'is_leader': self.is_leader,
            'running': self.running.is_set(),
            'gps_sync': {
                'offset': self.gps_sync.gps_time_offset,
                'last_sync': self.gps_sync.last_sync_time,
                'synced_time': self.gps_sync.get_synced_time(),
                'time_since_sync': time.time() - self.gps_sync.last_sync_time
            },
            'state_queue': self.state_queue.get_queue_stats(),
            'communication': {
                'send_port': self.send_port,
                'recv_port': self.recv_port,
                'ack_port': self.ack_port,
                'has_leader_state': self.leader_state is not None,
                'sequence_number': self.comm.sequence_number if hasattr(self.comm, 'sequence_number') else 0
            }
        }
        
        # Add leader state information if available
        if self.leader_state is not None:
            current_gps_time = self.gps_sync.get_synced_time()
            state_age = current_gps_time - self.leader_state.get('timestamp', 0)
            
            state['received_leader_state'] = {
                'sender_id': self.leader_state.get('id'),
                'position': self.leader_state.get('pos'),
                'velocity': self.leader_state.get('v'),
                'timestamp': self.leader_state.get('timestamp'),
                'age_seconds': state_age,
                'is_valid': state_age <= 1.0,
                'processing_delay': self.leader_state.get('processing_delay', 0.0)
            }
        
        # Add interpolated leader state info for followers
        if not self.is_leader and self.leader_vehicle:
            interpolated_state = self.get_interpolated_leader_state()
            if interpolated_state:
                state['interpolated_leader_state'] = {
                    'available': True,
                    'position': interpolated_state.get('pos'),
                    'velocity': interpolated_state.get('v'),
                    'interpolated': interpolated_state.get('interpolated', False),
                    'alpha': interpolated_state.get('alpha', 0.0),
                    'source_states': interpolated_state.get('source_states', [])
                }
            else:
                state['interpolated_leader_state'] = {'available': False}
        
        return state
