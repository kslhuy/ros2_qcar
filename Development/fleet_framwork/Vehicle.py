import threading
import time
import math
import logging
import numpy as np
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
from VehicleObserver import VehicleObserver

from src.GPS_sim.md_gps_sync import GPSSync
from md_logging_config import (
    get_individual_vehicle_logger, get_communication_logger, get_gps_logger, 
    get_control_logger, get_observer_logger, disable_all_logging, set_module_logging
)

# Import performance monitoring
try:
    from performance_monitor import perf_monitor
    PERFORMANCE_MONITORING = True
except ImportError:
    PERFORMANCE_MONITORING = False
    perf_monitor = None

class Vehicle:
    """
    Vehicle class that represents a single vehicle in the fleet.
    Each vehicle runs in its own thread and manages its own control logic.
    """
    
    def __init__(self, vehicle_id: int, qcar: QLabsQCar2, controller_type: str = "CACC", 
                 is_leader: bool = False, config=None, fleet_lock=None, 
                 target_ip: str = "127.0.0.1", base_send_port: int = 8000, 
                 base_recv_port: int = 9000, base_ack_port: int = 10000,
                 gps_server_ip: str = "127.0.0.1", gps_server_port: int = 8001,
                 fleet_size: int = 2, initial_pose: list = None):
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
            fleet_size: Total number of vehicles in the fleet
            initial_pose: Initial pose [x, y, z, roll, pitch, yaw] for the vehicle (default: None)
        """
        self.vehicle_id = vehicle_id
        self.qcar = qcar
        self.controller_type = controller_type
        self.is_leader = is_leader
        self.config = config
        self.fleet_lock = fleet_lock or threading.Lock()
        self.fleet_size = fleet_size
        
        # Thread management
        self.running = threading.Event()  # Changed to Event for better communication control
        self.control_thread = None
        self.observer_thread = None  # New observer thread
        self.send_thread = None
        self.receive_thread = None
        self.gps_sync_thread = None
        self.update_rate = 100  # Hz
        self.observer_rate = 100  # Hz - Observer update rate
        self.gps_update_rate = 50  # Hz - GPS data collection rate (separate from observer rate)
        
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
        self.observer_logger = get_observer_logger(vehicle_id)   # Observer timing and operations
        
        # GPS Time Synchronization
        self.gps_sync = GPSSync(gps_server_ip, gps_server_port, vehicle_id)
        self.sync_interval = 5.0  # Sync every 5 seconds
        self.last_sync_attempt = time.time()
        
        # State Queue for managing received states with time validation
        # Increased max_delay_threshold to 1.0s to account for processing delays
        self.state_queue = StateQueue(
            max_queue_size=50,
            max_age_seconds=2.0,           # Allow states up to 2 seconds old
            max_delay_threshold=1.0,       # Allow up to 1 second of processing delay
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
        
        # Initialize Vehicle Observer for state estimation
        self.observer = VehicleObserver(
            vehicle_id=self.vehicle_id,
            fleet_size=self.fleet_size,
            config=self.config,
            logger=self.logger
        )
        
        # Initialize EKF in observer with actual initial pose from QcarFleet
        # Use initial_pose parameter if provided, otherwise use default or config
        if initial_pose is not None and len(initial_pose) >= 6:
            # Extract position (x, y) and rotation (yaw) from initial_pose
            # initial_pose format: [x, y, z, roll, pitch, yaw]
            initial_ekf_pose = np.array([initial_pose[0], initial_pose[1], initial_pose[5]])  # [x, y, theta]
            self.logger.info(f"Vehicle {self.vehicle_id}: Using initial pose from QcarFleet - "
                           f"x={initial_pose[0]:.3f}, y={initial_pose[1]:.3f}, yaw={initial_pose[5]:.3f}")
        elif hasattr(self.config, 'initial_pose'):
            initial_ekf_pose = np.array(self.config.initial_pose[:3])
            self.logger.info(f"Vehicle {self.vehicle_id}: Using initial pose from config")
        else:
            initial_ekf_pose = np.array([0.0, 0.0, 0.0])  # [x, y, theta]
            self.logger.info(f"Vehicle {self.vehicle_id}: Using default initial pose")
        
        self.observer.initialize_ekf(initial_ekf_pose)
        
        # Initialize control input tracking for observer
        self.current_control_input = np.array([0.0, 0.0])  # [steering, acceleration]
        
        # Observer state cache for control loop access (thread-safe)
        self.observer_state_cache = {
            'position': [0.0, 0.0, 0.0],
            'rotation': [0.0, 0.0, 0.0],
            'velocity': 0.0,
            'gps_available': False,
            'ekf_initialized': False,
            'source': 'unknown',
            'timestamp': 0.0
        }
        self.observer_state_lock = threading.RLock()
        
        # GPS data cache for observer loop (separate from observer state)
        self.gps_data_cache = {
            'position': None,
            'rotation': None,
            'velocity': 0.0,
            'available': False,
            'timestamp': 0.0,
            'last_update': 0.0
        }
        self.gps_data_lock = threading.RLock()
        
        # Configure logging for optimal performance
        self.configure_logging_for_performance()

    def configure_logging_for_performance(self):
        """
        Configure logging settings for optimal performance.
        
        This method sets up logging to minimize performance overhead while
        maintaining essential observer timing information for analysis.
        """
        # Maximum performance - disable all logs by default
        disable_all_logging()
        
        # Enable only critical observer timing logs for performance monitoring
        # Reduce even observer logging frequency for better performance
        set_module_logging('observer', True)
        set_module_logging('timing', True)
        
        # Set logging level to WARNING for observer to reduce overhead
        self.observer_logger.setLevel(logging.WARNING)
        
        self.logger.info(f"Vehicle {self.vehicle_id}: Logging configured for performance - "
                        f"only observer and timing logs enabled")
        
    def configure_logging_for_development(self):
        """
        Configure logging for development and debugging.
        Enables comprehensive logging for troubleshooting.
        """
        from md_logging_config import enable_all_logging
        enable_all_logging()
        
        self.logger.info(f"Vehicle {self.vehicle_id}: Logging configured for development - "
                        f"all logs enabled")
        
    def configure_logging_custom(self, enabled_modules: list):
        """
        Configure custom logging modules.
        
        Args:
            enabled_modules: List of module names to enable 
                           (e.g., ['observer', 'timing', 'communication'])
        """
        disable_all_logging()
        
        for module in enabled_modules:
            set_module_logging(module, True)
            
        self.logger.info(f"Vehicle {self.vehicle_id}: Custom logging configured - "
                        f"enabled modules: {', '.join(enabled_modules)}")
    
    def configure_observer_rate(self, observer_rate: int, control_rate: int = None):
        """
        Configure observer and control update rates.
        
        Args:
            observer_rate: Observer update rate in Hz (recommended: 50-100 Hz)
            control_rate: Control loop rate in Hz (optional, defaults to current rate)
        """
        self.observer_rate = observer_rate
        if control_rate is not None:
            self.update_rate = control_rate
            
        self.logger.info(f"Vehicle {self.vehicle_id}: Observer rate set to {observer_rate} Hz, "
                        f"Control rate: {self.update_rate} Hz")
    
    def configure_gps_rate(self, gps_rate: int):
        """
        Configure GPS data collection rate independently of observer rate.
        
        Args:
            gps_rate: GPS update rate in Hz (recommended: 20-50 Hz for performance)
        """
        self.gps_update_rate = gps_rate
        self.logger.info(f"Vehicle {self.vehicle_id}: GPS update rate set to {gps_rate} Hz")
    
    def configure_rates(self, observer_rate: int = None, control_rate: int = None, gps_rate: int = None):
        """
        Configure all update rates at once.
        
        Args:
            observer_rate: Observer update rate in Hz (optional)
            control_rate: Control loop rate in Hz (optional)
            gps_rate: GPS data collection rate in Hz (optional)
        """
        if observer_rate is not None:
            self.observer_rate = observer_rate
        if control_rate is not None:
            self.update_rate = control_rate
        if gps_rate is not None:
            self.gps_update_rate = gps_rate
            
        self.logger.info(f"Vehicle {self.vehicle_id}: Rates configured - "
                        f"Observer: {self.observer_rate} Hz, "
                        f"Control: {self.update_rate} Hz, "
                        f"GPS: {self.gps_update_rate} Hz")
    
    def optimize_for_performance(self):
        """
        Apply performance optimizations for real-time operation.
        Reduces update rates and logging to improve performance.
        """
        # Reduce observer rate to 50Hz for better performance
        self.observer_rate = 50
        
        # Keep control rate at 50Hz to match observer
        self.update_rate = 50
        
        # Reduce GPS rate to 25Hz (still adequate for vehicle dynamics)
        self.gps_update_rate = 25
        
        # Configure minimal logging
        self.configure_logging_for_performance()
        
        # Set observer logger to WARNING level to reduce overhead
        self.observer_logger.setLevel(logging.WARNING)
        
        self.logger.warning(f"Vehicle {self.vehicle_id}: Performance optimization applied - "
                          f"Observer: {self.observer_rate}Hz, Control: {self.update_rate}Hz, GPS: {self.gps_update_rate}Hz")
        

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
    
    def update_gps_data(self):
        """
        Optimized GPS data collection method with minimal QLabs API calls.
        Updates GPS cache that the observer loop can use.
        """
        gps_start_time = time.perf_counter()
        gps_available = True
        
        try:
            # Optimize: Get GPS-like data from QLabs with minimal overhead
            # Use get_world_transform_fast() if available, or cache more aggressively
            try:
                _, pos, rot, _ = self.qcar.get_world_transform()
            except Exception as api_error:
                # If QLabs API is slow/failing, use cached data temporarily
                self.logger.warning(f"QLabs API slow/failed, using cached data: {api_error}")
                with self.gps_data_lock:
                    if self.gps_data_cache['available'] and self.gps_data_cache['position'] is not None:
                        # Use last known position with small prediction step
                        pos = self.gps_data_cache['position']
                        rot = self.gps_data_cache['rotation'] 
                        # Mark as degraded GPS
                        gps_available = False
                    else:
                        raise api_error  # No fallback available
            
            # Simulate GPS dropouts occasionally (optional)
            if hasattr(self.config, 'gps_dropout_probability'):
                import random
                if random.random() < self.config.gps_dropout_probability:
                    gps_available = False
            
            if gps_available:
                # Calculate velocity using GPS-synchronized time - optimized
                current_time = self.gps_sync.get_synced_time()
                gps_velocity = 0.0
                
                if self.prev_pos is not None and self.prev_time is not None:
                    dx = pos[0] - self.prev_pos[0]
                    dy = pos[1] - self.prev_pos[1]
                    dt = current_time - self.prev_time
                    if dt > 1e-6:
                        distance = math.sqrt(dx*dx + dy*dy)  # Slightly faster than **2
                        gps_velocity = distance / dt
                
                # Update GPS cache (thread-safe) - minimize lock time
                gps_data_update = {
                    'position': pos,
                    'rotation': rot,
                    'velocity': gps_velocity,
                    'available': True,
                    'timestamp': current_time,
                    'last_update': time.time()
                }
                
                with self.gps_data_lock:
                    self.gps_data_cache.update(gps_data_update)
                
                # Update stored GPS position and time for velocity calculation
                self.prev_pos = pos
                self.prev_time = current_time
                
            else:
                # Mark GPS as unavailable but keep timestamp - minimal update
                with self.gps_data_lock:
                    self.gps_data_cache.update({
                        'available': False,
                        'timestamp': self.gps_sync.get_synced_time(),
                        'last_update': time.time()
                    })
                
        except Exception as e:
            # Log warning only occasionally to reduce overhead
            if not hasattr(self, '_last_gps_error_log') or time.time() - self._last_gps_error_log > 5.0:
                self.logger.warning(f"GPS/QLabs data collection failed: {e}")
                self._last_gps_error_log = time.time()
            
            # Mark GPS as unavailable
            with self.gps_data_lock:
                self.gps_data_cache.update({
                    'available': False,
                    'timestamp': self.gps_sync.get_synced_time(),
                    'last_update': time.time()
                })
        
        gps_time = (time.perf_counter() - gps_start_time) * 1000
        
        # Log GPS collection performance only if very slow (reduce logging overhead)
        if gps_time > 50.0:  # Only log if more than 50ms (very slow)
            self.observer_logger.warning(f"GPS_COLLECTION: Very slow GPS collection ({gps_time:.3f}ms)")
        
        return gps_available, gps_time
    
    def get_cached_gps_data(self):
        """
        Get cached GPS data for observer use.
        
        Returns:
            tuple: (gps_available, measured_state, gps_age_ms)
        """
        try:
            with self.gps_data_lock:
                if not self.gps_data_cache['available']:
                    return False, None, 0.0
                
                # Check GPS data age
                current_time = time.time()
                gps_age = (current_time - self.gps_data_cache['last_update']) * 1000  # Convert to ms
                
                # If GPS data is too old (> 100ms), consider it unavailable
                max_gps_age_ms = 1000.0 / self.gps_update_rate * 2  # Allow up to 2 GPS cycles
                if gps_age > max_gps_age_ms:
                    return False, None, gps_age
                
                # Create measured_state from cached GPS data
                pos = self.gps_data_cache['position']
                rot = self.gps_data_cache['rotation']
                velocity = self.gps_data_cache['velocity']
                measured_state = np.array([pos[0], pos[1], rot[2], velocity])
                
                return True, measured_state, gps_age
                
        except Exception as e:
            self.logger.error(f"Error getting cached GPS data: {e}")
            return False, None, 0.0
    
    def observer_loop(self):
        """
        Optimized observer loop running in its own thread.
        Handles state estimation using cached GPS data and sensor fusion.
        Provides state estimates to the control loop via cached state.
        """
        self.logger.info(f"Vehicle {self.vehicle_id} observer loop started")
        
        # Track when to update GPS data - more efficient variables
        last_gps_update = 0.0
        gps_update_interval = 1.0 / self.gps_update_rate
        last_observer_time = None
        
        # Performance monitoring variables
        loop_count = 0
        last_performance_log = 0.0
        
        while self.running.is_set():
            start_time = time.time()
            
            try:
                # Start total timing for observer performance analysis
                observer_start_time = time.perf_counter()
                
                # Calculate actual elapsed time since last observer update (optimized)
                if last_observer_time is not None:
                    actual_observer_dt = start_time - last_observer_time
                    target_observer_dt = 1.0 / self.observer_rate
                    
                    # Only log timing issues occasionally (every 100 loops or if very bad)
                    timing_error = abs(actual_observer_dt - target_observer_dt) / target_observer_dt * 100
                    if timing_error > 50 or (timing_error > 20 and loop_count % 100 == 0):
                        self.observer_logger.warning(f"TIMING_DRIFT: Observer loop timing error {timing_error:.1f}% - "
                                                   f"actual={actual_observer_dt:.4f}s, target={target_observer_dt:.4f}s")
                else:
                    actual_observer_dt = 1.0 / self.observer_rate  # First iteration
                
                last_observer_time = start_time
                loop_count += 1
                
                # Minimize fleet_lock scope - only lock when absolutely necessary
                current_time = self.gps_sync.get_synced_time()
                
                # Step 1: GPS Data Collection (at configurable rate) - optimized logic
                gps_start_time = time.perf_counter()
                
                # More efficient GPS update check
                if current_time - last_gps_update >= gps_update_interval:
                    # Only take fleet lock when actually updating GPS
                    with self.fleet_lock:
                        gps_available, gps_collection_time = self.update_gps_data()
                    last_gps_update = current_time
                    fresh_gps = True
                    
                    # Get the fresh GPS data that was just collected
                    if gps_available:
                        _, measured_state, gps_age = self.get_cached_gps_data()
                    else:
                        measured_state = None
                        gps_age = 0.0
                else:
                    gps_collection_time = 0.0
                    fresh_gps = False
                    gps_available = False
                    measured_state = None
                    gps_age = 0.0
                
                gps_time = (time.perf_counter() - gps_start_time) * 1000
                
                # Step 2: Sensor Data Collection (minimal overhead)
                sensor_start_time = time.perf_counter()
                try:
                    with self.observer_state_lock:
                        motor_tach = self.observer_state_cache['velocity']
                    gyroscope_z = 0.0  # Simulated gyro
                except Exception:
                    motor_tach = 0.0
                    gyroscope_z = 0.0
                
                sensor_time = (time.perf_counter() - sensor_start_time) * 1000
                
                # Step 3: Observer Local State Update (main computation)
                observer_update_start = time.perf_counter()
                estimated_state = self.observer.update_local_state(
                    measured_state=measured_state,
                    control_input=self.current_control_input, 
                    timestamp=current_time,
                    motor_tach=motor_tach,
                    gyroscope_z=gyroscope_z
                )
                observer_update_time = (time.perf_counter() - observer_update_start) * 1000
                
                # Step 4: Get Estimated State for Control (fast)
                control_state_start = time.perf_counter()
                estimated_state_info = self.observer.get_estimated_state_for_control()
                control_state_time = (time.perf_counter() - control_state_start) * 1000
                
                # Step 5: Update Cached State for Control Loop (optimized)
                state_update_start = time.perf_counter()
                with self.observer_state_lock:
                    # Batch update for better performance
                    cache_update = {
                        'position': estimated_state_info['position'].copy(),
                        'rotation': estimated_state_info['rotation'].copy(),
                        'velocity': estimated_state_info['velocity'],
                        'gps_available': estimated_state_info['gps_available'],
                        'ekf_initialized': estimated_state_info['ekf_initialized'],
                        'source': estimated_state_info['source'],
                        'timestamp': current_time
                    }
                    self.observer_state_cache.update(cache_update)
                    
                    # Also update the legacy vehicle state for compatibility
                    self.current_pos = estimated_state_info['position'].copy()
                    self.current_rot = estimated_state_info['rotation'].copy()
                    self.velocity = estimated_state_info['velocity']
                
                state_update_time = (time.perf_counter() - state_update_start) * 1000
                
                # Step 6: Distributed Observer Update (if needed)
                distributed_start = time.perf_counter()
                self.observer.update_distributed_estimates(current_time)
                distributed_time = (time.perf_counter() - distributed_start) * 1000
                
                # Calculate total observer time
                total_observer_time = (time.perf_counter() - observer_start_time) * 1000
                
                # Optimized logging - only log detailed timing occasionally
                current_log_time = time.time()
                should_log_details = (
                    total_observer_time > 15.0 or  # Always log if very slow
                    (current_log_time - last_performance_log > 2.0 and total_observer_time > 5.0) or  # Log every 2s if moderate
                    loop_count % 500 == 0  # Log every 500 loops regardless
                )
                
                if should_log_details:
                    target_dt = 1.0 / self.observer_rate
                    self.observer_logger.info(f"TIMING: Total={total_observer_time:.3f}ms, GPS={gps_time:.3f}ms, "
                                            f"Sensor={sensor_time:.3f}ms, ObsUpdate={observer_update_time:.3f}ms, "
                                            f"ControlState={control_state_time:.3f}ms, StateUpdate={state_update_time:.3f}ms, "
                                            f"Distributed={distributed_time:.3f}ms, "
                                            f"ActualDt={actual_observer_dt:.4f}s, TargetDt={target_dt:.4f}s")
                    last_performance_log = current_log_time
                
                # Reduced GPS logging frequency
                if fresh_gps and should_log_details:
                    self.observer_logger.info(f"GPS_COLLECTION: Fresh GPS collected in {gps_collection_time:.3f}ms, "
                                            f"Available={gps_available}, Rate={self.gps_update_rate}Hz")
                    
                    if gps_available and measured_state is not None:
                        self.observer_logger.info(f"GPS: Pos=({measured_state[0]:.3f},{measured_state[1]:.3f}), "
                                                f"Theta={measured_state[2]:.3f}, Vel={measured_state[3]:.3f}")
                    
                    self.observer_logger.info(f"CONTROL_INPUT: Steering={self.current_control_input[0]:.3f}, "
                                            f"Accel={self.current_control_input[1]:.3f}")
                
                # Performance warning only for severe issues
                if total_observer_time > 15.0:  # More than 15ms (very bad)
                    self.observer_logger.warning(f"PERFORMANCE: Observer update very slow ({total_observer_time:.3f}ms) - "
                                               f"Target: <10ms for {self.observer_rate}Hz observer loop")
                
                # Maintain observer update rate - optimized sleep calculation
                elapsed = time.time() - start_time
                target_cycle_time = 1.0 / self.observer_rate
                sleep_time = max(0, target_cycle_time - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
            except Exception as e:
                total_time = (time.perf_counter() - observer_start_time) * 1000 if 'observer_start_time' in locals() else 0
                self.observer_logger.error(f"ERROR: Observer loop failed after {total_time:.3f}ms - {e}")
                time.sleep(0.01)  # Brief pause before retry
        
        self.logger.info(f"Vehicle {self.vehicle_id} observer loop exited")

    def update_state(self):
        """
        Simplified state update method for backwards compatibility.
        Now just returns cached observer state since observer runs in separate thread.
        """
        try:
            with self.observer_state_lock:
                # Return cached state from observer thread
                return {
                    'position': self.observer_state_cache['position'].copy(),
                    'rotation': self.observer_state_cache['rotation'].copy(),
                    'velocity': self.observer_state_cache['velocity'],
                    'gps_available': self.observer_state_cache['gps_available'],
                    'source': self.observer_state_cache['source']
                }
        except Exception as e:
            self.logger.error(f"Error getting cached state: {e}")
            # Fallback to current vehicle state
            return {
                'position': self.current_pos.copy(),
                'rotation': self.current_rot.copy(),
                'velocity': self.velocity,
                'gps_available': False,
                'source': 'fallback'
            }

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
        Optimized state processing for better performance.
        This method should be called by CommHandler when new states are received.
        
        Args:
            received_state: State data received from another vehicle
        """
        # Start performance timing
        if PERFORMANCE_MONITORING and perf_monitor:
            start_time = perf_monitor.start_timing()
        
        try:
            # Fast path: Skip validation for followers receiving leader data if performance critical
            sender_id = received_state.get('id')
            
            # Extract state and control information for observer
            if 'pos' in received_state and 'rot' in received_state and 'v' in received_state:
                pos = received_state['pos']
                rot = received_state['rot']
                velocity = received_state['v']
                timestamp = received_state.get('timestamp', self.gps_sync.get_synced_time())
                
                # Convert to observer format: [x, y, theta, v]
                state_vector = np.array([pos[0], pos[1], rot[2], velocity])
                
                # Extract control input if available, otherwise use zeros
                control_vector = np.array(received_state.get('control', [0.0, 0.0]))
                
                # Add to observer's distributed estimation
                self.observer.add_received_state(sender_id, state_vector, control_vector, timestamp)
            
            # Quick validation for critical path optimization
            if (not self.is_leader and 
                self.leader_vehicle and 
                sender_id == self.leader_vehicle.vehicle_id):
                
                # Fast path: Direct state update for leader-follower communication
                # Still validate through StateQueue but optimize for performance
                if self.state_queue.add_state(received_state, self.gps_sync):
                    # Directly update leader_state without additional queue lookup
                    self.leader_state = received_state
                    if PERFORMANCE_MONITORING and perf_monitor:
                        perf_monitor.end_timing(start_time, "processing")
                    return
            
            # Standard path: Full validation and processing
            if self.state_queue.add_state(received_state, self.gps_sync):
                # Only do expensive operations if state was accepted
                if not self.is_leader:
                    # Get the most recent valid state from the leader
                    latest_leader_state = self.state_queue.get_latest_valid_state(
                        sender_id=self.leader_vehicle.vehicle_id if self.leader_vehicle else None
                    )
                    
                    if latest_leader_state:
                        self.leader_state = latest_leader_state
            else:
                # Only generate detailed stats if logging level requires it
                if self.logger.isEnabledFor(logging.WARNING):
                    stats = self.state_queue.get_queue_stats()
                    self.logger.warning(f"Vehicle {self.vehicle_id}: Invalid state rejected from vehicle {sender_id} "
                                      f"(Rejected: {stats['total_received'] - stats['valid_states']}, "
                                      f"Expired: {stats['expired_states']}, Delayed: {stats['delayed_states']}, "
                                      f"Duplicates: {stats['duplicate_states']})")
                
        except Exception as e:
            self.logger.error(f"Error processing received state: {e}")
        finally:
            # End performance timing
            if PERFORMANCE_MONITORING and perf_monitor:
                perf_monitor.end_timing(start_time, "processing")
    
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
            # Get state for control (uses observer estimates when GPS not available)
            control_state = self.get_state_for_control()
            
            # Calculate actual time elapsed since last control update
            current_time = time.time()
            if not hasattr(self, '_last_control_time'):
                self._last_control_time = current_time
                dt = 1.0 / self.update_rate  # Use nominal dt for first iteration
            else:
                dt = current_time - self._last_control_time
                # Clamp dt to reasonable bounds to avoid instability
                dt = max(0.001, min(dt, 0.1))  # Between 1ms and 100ms
            
            self._last_control_time = current_time
            
            # Compute control commands using the dedicated leader controller
            # Use observer estimated state for more robust control
            forward_speed, steering_angle = self.leader_controller.compute_control(
                current_pos=control_state['position'],
                current_rot=control_state['rotation'],
                velocity=control_state['velocity'],
                dt=dt  # Use actual elapsed time
            )
            
            # Update control input for observer
            # Convert controller outputs to observer format [steering, acceleration]
            self.current_control_input = np.array([steering_angle, forward_speed])
            
            # Log control timing for debugging
            if self.logger.isEnabledFor(logging.DEBUG):
                target_dt = 1.0 / self.update_rate
                timing_error = abs(dt - target_dt) / target_dt * 100
                self.logger.debug(f"Leader control using {control_state['source']} - "
                                f"GPS available: {control_state['gps_available']}, "
                                f"dt={dt:.4f}s (target={target_dt:.4f}s, error={timing_error:.1f}%)")
                
        except Exception as e:
            self.logger.error(f"Leader control error: {e}")
            # Reset control input on error
            self.current_control_input = np.array([0.0, 0.0])
 
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
                        # Update control input for observer
                        self.current_control_input = np.array([0.0, 0.0])
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
                    # Update control input for observer
                    self.current_control_input = np.array([0.0, 0.0])
                    with self.fleet_lock:
                        self.qcar.set_velocity_and_request_state(
                            forward=0.0, turn=0.0, headlights=False,
                            leftTurnSignal=False, rightTurnSignal=False,
                            brakeSignal=False, reverseSignal=False
                        )
                    return

                # Get state for control (uses observer estimates when GPS not available)
                control_state = self.get_state_for_control()
                
                # Calculate actual time elapsed since last control update
                current_time = time.time()
                if not hasattr(self, '_last_control_time'):
                    self._last_control_time = current_time
                    dt = 1.0 / self.update_rate  # Use nominal dt for first iteration
                else:
                    dt = current_time - self._last_control_time
                    # Clamp dt to reasonable bounds to avoid instability
                    dt = max(0.001, min(dt, 0.1))  # Between 1ms and 100ms
                
                self._last_control_time = current_time
                
                # Compute control commands using the dedicated follower controller
                forward_speed, steering_angle = self.follower_controller.compute_control(
                    current_pos=control_state['position'],
                    current_rot=control_state['rotation'],
                    velocity=control_state['velocity'],
                    dt=dt,  # Use actual elapsed time
                    leader_data=leader_data  # Pass validated leader data to controller
                )
                
                # Update control input for observer
                # Convert controller outputs to observer format [steering, acceleration]
                self.current_control_input = np.array([steering_angle, forward_speed])
                
                # Log control source for debugging
                if self.logger.isEnabledFor(logging.DEBUG):
                    self.logger.debug(f"Follower control using {control_state['source']} - "
                                    f"GPS available: {control_state['gps_available']}")
                
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
                # Reset control input on error
                self.current_control_input = np.array([0.0, 0.0])
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
        """
        Simplified control loop running in its own thread.
        Gets state from observer cache instead of doing expensive state updates.
        """
        self.logger.info(f"Vehicle {self.vehicle_id} control loop started (Leader: {self.is_leader})")
        
        while self.running.is_set():
            start_time = time.time()
            
            try:
                # No need to call update_state() anymore - observer thread handles it
                # Just execute control logic based on role using cached state
                if self.is_leader:
                    self.leader_control_logic()
                else:
                    self.follower_control_logic()
                
                # Maintain update rate
                elapsed = time.time() - start_time
                sleep_time = max(0, (1.0 / self.update_rate) - elapsed)
                time.sleep(sleep_time)
                
            except Exception as e:
                self.logger.error(f"Control loop error: {e}")
                time.sleep(0.1)  # Brief pause before retry
        
        self.logger.info(f"Vehicle {self.vehicle_id} control loop exited")
    
    def start(self):
        """Start the vehicle's observer thread, control thread and communication threads."""
        if self.control_thread is None or not self.control_thread.is_alive():
            self.running.set()  # Set the event flag
            
            # Start GPS synchronization thread first
            self.gps_sync_thread = threading.Thread(target=self.gps_sync_loop, daemon=True)
            self.gps_sync_thread.start()
            self.logger.info(f"Vehicle {self.vehicle_id} GPS sync thread started")
            
            # Small delay to allow initial GPS sync
            time.sleep(0.2)
            
            # Start observer thread - this handles all state estimation
            self.observer_thread = threading.Thread(target=self.observer_loop, daemon=True)
            self.observer_thread.start()
            self.logger.info(f"Vehicle {self.vehicle_id} observer thread started")
            
            # Small delay to allow observer to initialize
            time.sleep(0.2)

            # Start communication threads
            self.send_thread = threading.Thread(target=self.comm.send_state, daemon=True)
            self.receive_thread = threading.Thread(target=self.comm.receive_messages, daemon=True)
            self.logger.debug("Starting send and receive threads")
            
            # Start control thread - now lightweight since observer runs separately
            self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
            self.control_thread.start()
            
            self.send_thread.start()
            time.sleep(0.1)  # Small delay to stagger thread startup
            self.receive_thread.start()
            
            self.logger.info(f"Vehicle {self.vehicle_id} started with all threads (GPS sync, observer, control, communication)")
    
    def stop(self):
        """Stop the vehicle's observer thread, control thread and communication threads."""
        self.running.clear()  # Clear the event flag
        
        # Stop observer thread
        if self.observer_thread is not None:
            self.observer_thread.join(timeout=1.0)
        
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
        
        # Reset observer
        try:
            self.observer.reset_observer()
        except Exception as e:
            self.logger.error(f"Error resetting observer: {e}")
            
        self.logger.info(f"Vehicle {self.vehicle_id} stopped")
    
    def is_alive(self):
        """Check if the vehicle's observer thread, control thread and communication threads are alive."""
        control_alive = self.control_thread is not None and self.control_thread.is_alive()
        observer_alive = self.observer_thread is not None and self.observer_thread.is_alive()
        send_alive = self.send_thread is not None and self.send_thread.is_alive()
        receive_alive = self.receive_thread is not None and self.receive_thread.is_alive()
        
        return control_alive and observer_alive and self.running.is_set()
    
    def join(self, timeout=None):
        """Wait for the vehicle's control thread to finish."""
        if self.control_thread is not None:
            self.control_thread.join(timeout)
    
    def get_state(self , detailed: bool = False) -> dict:
        """Get current vehicle state with enhanced information."""
        # Get basic observer state information
        observer_state_info = self.observer.get_estimated_state_for_control()
        
        state = {
            'vehicle_id': self.vehicle_id,
            'position': self.current_pos.copy(),
            'rotation': self.current_rot.copy(),
            'velocity': self.velocity,
            'is_leader': self.is_leader,
            'running': self.running.is_set(),
            'observer_state': {
                'estimated_position': observer_state_info['position'],
                'estimated_rotation': observer_state_info['rotation'],
                'estimated_velocity': observer_state_info['velocity'],
                'gps_available': observer_state_info['gps_available'],
                'ekf_initialized': observer_state_info['ekf_initialized'],
                'time_since_gps': observer_state_info['time_since_gps']
            },
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
        
        if detailed:
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
            
            # Add performance monitoring status
            if PERFORMANCE_MONITORING and perf_monitor:
                state['performance_monitoring'] = {
                    'enabled': True,
                    'integration_status': perf_monitor.get_integration_status(),
                    'recent_stats': perf_monitor.get_performance_stats()
                }
            else:
                state['performance_monitoring'] = {'enabled': False}
            
            # Add observer information
            try:
                observer_stats = self.observer.get_observer_stats()
                state['observer'] = {
                    'local_state_estimate': observer_stats['local_state'].tolist(),
                    'fleet_state_estimates': observer_stats['fleet_states'].tolist(),
                    'observer_config': observer_stats['config'],
                    'received_data_counts': observer_stats['received_data_counts'],
                    'estimation_log_size': observer_stats['estimation_log_size'],
                    'validation_log_size': observer_stats['validation_log_size']
                }
                
                # Add Kalman filter specific info if available
                if 'kalman_covariance' in observer_stats:
                    state['observer']['kalman_trace'] = observer_stats['kalman_trace']
                    state['observer']['covariance_determinant'] = np.linalg.det(observer_stats['kalman_covariance'])
                    
            except Exception as e:
                self.logger.error(f"Error getting observer stats: {e}")
                state['observer'] = {'error': str(e)}
        
        return state
    
    def get_state_for_control(self) -> dict:
        """
        Get vehicle state optimized for control algorithms.
        Now uses cached observer state from observer thread for better performance.
        
        Returns:
            Dictionary with state suitable for controllers
        """
        try:
            with self.observer_state_lock:
                # Use cached observer state from observer thread
                return {
                    'position': self.observer_state_cache['position'].copy(),
                    'rotation': self.observer_state_cache['rotation'].copy(),
                    'velocity': self.observer_state_cache['velocity'],
                    'source': self.observer_state_cache['source'],
                    'gps_available': self.observer_state_cache['gps_available'],
                    'ekf_initialized': self.observer_state_cache['ekf_initialized'],
                    'time_since_gps': time.time() - self.observer_state_cache['timestamp']
                }
        except Exception as e:
            self.logger.error(f"Error getting cached observer state: {e}")
            # Fallback to direct observer query (slower)
            observer_state = self.observer.get_estimated_state_for_control()
            return {
                'position': observer_state['position'],
                'rotation': observer_state['rotation'], 
                'velocity': observer_state['velocity'],
                'source': 'observer_fallback',
                'gps_available': observer_state['gps_available'],
                'ekf_initialized': observer_state['ekf_initialized'],
                'time_since_gps': observer_state.get('time_since_gps', 0.0)
            }
    
    def get_observer_performance_stats(self):
        """Get performance statistics for the observer."""
        try:
            # This could be extended to get actual timing stats from observer
            observer_state = self.observer.get_estimated_state_for_control()
            return {
                'vehicle_id': self.vehicle_id,
                'observer_initialized': hasattr(self, 'observer') and self.observer is not None,
                'ekf_status': observer_state['ekf_initialized'],
                'gps_available': observer_state['gps_available'],
                'time_since_gps': observer_state.get('time_since_gps', 0.0),
                'state_source': observer_state['source'],
                'last_update_time': time.time(),
                'control_loop_rate': self.update_rate,
                'is_leader': self.is_leader
            }
        except Exception as e:
            self.logger.error(f"Error getting observer stats: {e}")
            return {'vehicle_id': self.vehicle_id, 'error': str(e)}
    
    def initialize_observer_with_current_pose(self):
        """Initialize observer EKF with current vehicle pose if available."""
        try:
            # Try to get current pose from QLabs
            _, pos, rot, _ = self.qcar.get_world_transform()
            initial_pose = np.array([pos[0], pos[1], rot[2]])  # [x, y, theta]
            self.observer.initialize_ekf(initial_pose)
            self.logger.info(f"Observer initialized with current pose: {initial_pose}")
        except Exception as e:
            self.logger.warning(f"Could not initialize observer with current pose: {e}")
            # Fallback to default initialization
            self.observer.initialize_ekf()
    
    def get_local_state_estimate(self) -> np.ndarray:
        """Get the local state estimate from the observer."""
        return self.observer.get_local_state()
    
    def get_fleet_state_estimates(self) -> np.ndarray:
        """Get all fleet state estimates from the observer."""
        return self.observer.get_fleet_states()
    
    def get_vehicle_state_estimate(self, vehicle_id: int) -> Optional[np.ndarray]:
        """Get the state estimate for a specific vehicle."""
        return self.observer.get_vehicle_state(vehicle_id)
    
    def validate_observer_estimate(self, vehicle_id: int, true_state: np.ndarray) -> dict:
        """Validate observer estimate against ground truth."""
        return self.observer.validate_state_estimate(vehicle_id, true_state)
    
    def get_observer_performance(self) -> dict:
        """Get observer performance statistics."""
        return self.observer.get_observer_stats()
