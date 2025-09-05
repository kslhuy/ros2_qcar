import multiprocessing
import time
import math
import logging
import numpy as np
import os
import sys
import socket
import json
from typing import Any, Optional, Dict, List

# QLabs imports - these will be created inside each process
from qvl.qlabs import QuanserInteractiveLabs
from qvl.qcar2 import QLabsQCar2
from pal.utilities.math import wrap_to_pi

from qvl.real_time import QLabsRealTime

# Physical QCar imports (for EKF and GPS mode)
try:
    from pal.products.qcar import QCar, QCarGPS, IS_PHYSICAL_QCAR
    from hal.content.qcar_functions import QCarEKF
    PHYSICAL_QCAR_AVAILABLE = True
except ImportError:
    PHYSICAL_QCAR_AVAILABLE = False
    QCar = None
    QCarGPS = None
    QCarEKF = None
    IS_PHYSICAL_QCAR = False

# Controller imports
from src.Controller.CACC import CACC
from src.Controller.idm_control import IDMControl
from src.Controller.DummyController import DummyController, DummyVehicle

# Other imports
from CommHandler import CommHandler
from VehicleLeaderController import VehicleLeaderController
from VehicleFollowerController import VehicleFollowerController
from StateQueue import StateQueue
from VehicleObserver import VehicleObserver

from src.GPS_sim.md_gps_sync import GPSSync
from md_logging_config import (
    get_individual_vehicle_logger, get_communication_logger, get_gps_logger, 
    get_control_logger, get_observer_logger, get_fleet_observer_logger, 
    disable_all_logging, set_module_logging
)

# Import performance monitoring
try:
    from performance_monitor import perf_monitor
    PERFORMANCE_MONITORING = True
except ImportError:
    PERFORMANCE_MONITORING = False
    perf_monitor = None


def vehicle_process_main(vehicle_config: Dict, stop_event: multiprocessing.Event, 
                        status_queue: multiprocessing.Queue):
    """
    Main function that runs in each vehicle process.
    
    Args:
        vehicle_config: Dictionary containing all vehicle configuration
        stop_event: Multiprocessing event to signal process shutdown
        status_queue: Queue for sending status updates to main process
    """
    
    # Extract configuration
    vehicle_id = vehicle_config['vehicle_id']
    controller_type = vehicle_config['controller_type']
    is_leader = vehicle_config['is_leader']
    # fleet_size = vehicle_config['fleet_size']
    # initial_pose = vehicle_config['initial_pose']
    
    # # Communication config
    # target_ip = vehicle_config['target_ip']
    # send_port = vehicle_config['send_port']
    # recv_port = vehicle_config['recv_port']
    # ack_port = vehicle_config['ack_port']
    
    # # GPS config
    # gps_server_ip = vehicle_config['gps_server_ip']
    # gps_server_port = vehicle_config['gps_server_port']
    
    # # Control parameters
    # max_steering = vehicle_config.get('max_steering', 0.6)
    # lookahead_distance = vehicle_config.get('lookahead_distance', 7.0)
    # update_rate = vehicle_config.get('update_rate', 100)
    # observer_rate = vehicle_config.get('observer_rate', 100)
    # gps_update_rate = vehicle_config.get('gps_update_rate', 50)
    
    # Vehicle scale and spawn info
    vehicle_scale = vehicle_config['vehicle_scale']
    spawn_location = vehicle_config['spawn_location']
    spawn_rotation = vehicle_config['spawn_rotation']
    
    try:
        # Create QLabs connection inside this process
        qlabs = QuanserInteractiveLabs()
        try:
            qlabs.open("localhost")
            print(f"Vehicle {vehicle_id}: Connected to QLabs")
        except Exception as e:
            print(f"Vehicle {vehicle_id}: Error connecting to QLabs: {e}")
            status_queue.put({'vehicle_id': vehicle_id, 'status': 'failed', 'error': str(e)})
            return
        
        # Create QCar object inside this process
        qcar = QLabsQCar2(qlabs)
        
        # Spawn or connect to the vehicle
        try:
            success = qcar.spawn_id(
                actorNumber=vehicle_id,
                location=spawn_location,
                rotation=spawn_rotation,
                scale=vehicle_scale
            )
            time.sleep(0.2)  # Allow some time for the vehicle to spawn

            # print(f"Vehicle {vehicle_id}: Spawn success: {success}")
            print(f"Vehicle {vehicle_id}: Spawn parameters - location: {spawn_location}, rotation: {spawn_rotation}, scale: {vehicle_scale}")
            
            # if not success:
            #     print(f"Vehicle {vehicle_id}: Failed to spawn vehicle - spawn_id returned False")
                # Don't return here, continue with the process as the vehicle might still be usable
            
        except Exception as spawn_error:
            print(f"Vehicle {vehicle_id}: Exception during spawn: {spawn_error}")
            # Continue anyway, the vehicle might still work
        
        print(f"Vehicle {vehicle_id}: Successfully spawned at {spawn_location}")
        
        # Create the vehicle instance
        vehicle = VehicleProcess(
            vehicle_id=vehicle_id,
            qcar=qcar,
            qlabs=qlabs,
            controller_type=controller_type,
            is_leader=is_leader,
            vehicle_config=vehicle_config,
            stop_event=stop_event,
            status_queue=status_queue
        )
        
        # Signal successful initialization
        status_queue.put({'vehicle_id': vehicle_id, 'status': 'initialized'})
        
        # Run the vehicle with emergency stopping
        try:
            vehicle.run()
        except KeyboardInterrupt:
            print(f"Vehicle {vehicle_id}: KeyboardInterrupt caught, stopping immediately")
        except Exception as e:
            print(f"Vehicle {vehicle_id}: Unexpected error during run: {e}")
        finally:
            # EMERGENCY STOP: Ensure vehicle is stopped before process exits
            try:
                if hasattr(vehicle, 'qcar') and vehicle.qcar is not None:
                    vehicle.qcar.set_velocity_and_request_state(forward=0.0, turn=0.0 ,headlights=False,
                            leftTurnSignal=False,
                            rightTurnSignal=False,
                            brakeSignal=False,
                            reverseSignal=False
                    )
                    print(f"Vehicle {vehicle_id}: Emergency stop - Virtual QCar stopped")
                if hasattr(vehicle, 'physical_qcar') and vehicle.physical_qcar is not None:
                    vehicle.physical_qcar.write(0, 0)
                    print(f"Vehicle {vehicle_id}: Emergency stop - Physical QCar stopped")
            except Exception as stop_error:
                print(f"Vehicle {vehicle_id}: Error during emergency stop: {stop_error}")
        
        # After run() exits, call stop to clean up
        vehicle.stop()
        
    except Exception as e:
        print(f"Vehicle {vehicle_id}: Process error: {e}")
        status_queue.put({'vehicle_id': vehicle_id, 'status': 'error', 'error': str(e)})
    finally:
        print(f"Vehicle {vehicle_id}: Process shutting down")
        status_queue.put({'vehicle_id': vehicle_id, 'status': 'shutdown'})


class VehicleProcess:
    """
    Vehicle class that runs in its own process and manages its own control logic.
    Each process creates its own QLabs connection and QCar object.
    """
    
    def __init__(self, vehicle_id: int, qcar: QLabsQCar2, qlabs: QuanserInteractiveLabs,
                 controller_type: str = "CACC", is_leader: bool = False, 
                 vehicle_config: Dict = None, stop_event: multiprocessing.Event = None,
                 status_queue: multiprocessing.Queue = None):
        """
        Initialize a vehicle process instance.
        
        Args:
            vehicle_id: Unique identifier for this vehicle
            qcar: QLabsQCar2 instance for this vehicle (created in this process)
            qlabs: QuanserInteractiveLabs instance (created in this process)
            controller_type: Type of controller ("CACC" or "IDM")
            is_leader: Whether this vehicle is the leader
            vehicle_config: Configuration dictionary
            stop_event: Multiprocessing event for shutdown signaling
            status_queue: Queue for sending status updates
        """
        self.vehicle_id = vehicle_id
        self.qcar = qcar
        self.qlabs = qlabs
        self.controller_type = controller_type
        self.is_leader = is_leader
        self.config = vehicle_config
        self.stop_event = stop_event
        self.status_queue = status_queue
        
        # Control parameters (needed early for physical QCar initialization)
        self.max_steering = vehicle_config.get('max_steering', 0.6)
        self.lookahead_distance = vehicle_config.get('lookahead_distance', 7.0)
        self.k_steering = 2.0
        self.update_rate = vehicle_config.get('update_rate', 100)
        self.observer_rate = vehicle_config.get('observer_rate', 100)
        self.gps_update_rate = vehicle_config.get('gps_update_rate', 10)
        
        # QCar mode configuration - check if we should use physical QCar API for leader
        # TODO : instead of use is_leader , depend on config file if we need to use that 
        self.use_control_observer_mode = vehicle_config.get('use_control_observer_mode', False) 

        self.use_physical_qcar = vehicle_config.get('use_physical_qcar', False) and is_leader and PHYSICAL_QCAR_AVAILABLE
        print(f"Vehicle {vehicle_id}: use_physical_qcar={self.use_physical_qcar}, PHYSICAL_QCAR_AVAILABLE={PHYSICAL_QCAR_AVAILABLE}, is_leader={is_leader}")
        self.enable_steering_control = vehicle_config.get('enable_steering_control', True)
        self.calibrate = vehicle_config.get('calibrate', False)
        
        # Initialize QCar interfaces based on mode
        self.physical_qcar = None
        
        if self.use_physical_qcar:
            try:
                self.rtModel = os.path.normpath(os.path.join(os.environ['RTMODELS_DIR'], 'QCar2/QCar2_Workspace_studio'))
                QLabsRealTime().start_real_time_model(self.rtModel, actorNumber=vehicle_id)

                # Initialize physical QCar interface for leader
                self.physical_qcar = QCar(readMode=1, frequency=self.update_rate)
                
                # Initialize GPS for EKF if steering control is enabled
                if self.enable_steering_control:
                    try:
                        self.gps = QCarGPS(readMode=1, frequency=self.gps_update_rate)
                        # Initialize EKF for state estimation
                        self.ekf = QCarEKF(
                            xhat_0=np.array([0, 0, 0, 0]),  # Initial state [x, y, yaw, v]
                            P_0=0.01*np.eye(4),             # Initial covariance
                            Q=np.diag([0.1, 0.1, 0.1, 0.1]), # Process noise
                            R=np.diag([0.5, 0.5, 0.1])       # Measurement noise [x, y, yaw]
                        )
                        self.ekf_initialized = False
                        print(f"Vehicle {self.vehicle_id}: Physical QCar with GPS and EKF initialized")
                    except Exception as gps_error:
                        print(f"Vehicle {self.vehicle_id}: GPS initialization failed: {gps_error}")
                        self.gps = None
                        self.ekf = None
                        self.ekf_initialized = False
                else:
                    self.gps = None
                    self.ekf = None
                    self.ekf_initialized = False
                    print(f"Vehicle {self.vehicle_id}: Physical QCar initialized without GPS/EKF")
                    
            except Exception as e:
                print(f"Vehicle {self.vehicle_id}: Failed to initialize physical QCar: {e}")
                self.use_physical_qcar = False
                self.physical_qcar = None
                self.gps = None
                self.ekf = None
                self.ekf_initialized = False


        self.stop_event = stop_event
        self.status_queue = status_queue
        
        # Multiprocessing-safe running flag
        self.running = multiprocessing.Event()
        self.running.set()
        
        # Extract configuration values
        self.fleet_size = vehicle_config.get('fleet_size', 2)
        self.target_ip = vehicle_config['target_ip']
        self.send_port = vehicle_config['send_port']
        self.recv_port = vehicle_config['recv_port']
        self.ack_port = vehicle_config['ack_port']
        
        # NEW: Chain-following configuration
        self.following_target = vehicle_config.get('following_target', None)
        if self.following_target is not None:
            print(f"Vehicle {self.vehicle_id}: Configured to follow vehicle {self.following_target}")
        else:
            print(f"Vehicle {self.vehicle_id}: Leader vehicle (no following target)")
        
        # Vehicle state - initialize with spawn location and rotation
        spawn_location = vehicle_config.get('spawn_location', [0, 0, 0])
        spawn_rotation = vehicle_config.get('spawn_rotation', [0, 0, 0])
        self.current_pos = list(spawn_location)
        self.current_rot = list(spawn_rotation)
        self.velocity = 0.0
        self.prev_pos = None
        self.prev_time = None
        
        # Initialize logging for this process
        self.logger = get_individual_vehicle_logger(vehicle_id)
        self.comm_logger = get_communication_logger(vehicle_id)
        self.gps_logger = get_gps_logger(vehicle_id)
        self.control_logger = get_control_logger(vehicle_id)
        self.observer_logger = get_observer_logger(vehicle_id)
        self.fleet_observer_logger = get_fleet_observer_logger(vehicle_id)
        
        # Configure logging for performance
        self.configure_logging_for_performance()
        
        # GPS Time Synchronization
        gps_server_ip = vehicle_config.get('gps_server_ip', '127.0.0.1')
        gps_server_port = vehicle_config.get('gps_server_port', 8001)
        self.gps_sync = GPSSync(gps_server_ip, gps_server_port, vehicle_id)
        self.sync_interval = 5.0
        self.last_sync_attempt = time.time()
        
        # Local simulation (high rate, low jitter):

        # broadcast_rate ≈ 100 Hz
        # max_age_seconds: 0.35–0.40
        # max_delay_threshold: 0.12
        # max_queue_size: 64 ( (100 * 0.4) *1.2 = 48 → choose power-of-two 64)
        # Physical lab LAN / moderate jitter (~40–60 Hz effective):

        # max_age_seconds: 0.5
        # max_delay_threshold: 0.18–0.20
        # max_queue_size: 64–80 (choose 80 if not power-of-two constraint)
        # Higher latency / experimental:

        # max_age_seconds: 0.8–1.0
        # max_delay_threshold: 0.30–0.40
        # max_queue_size: 100–120 (if rate ~100 Hz) or 60 (if rate ~60 Hz)
        
        # State Queue for managing received states
        self.state_queue = StateQueue(
            max_queue_size=64,
            max_age_seconds=0.4,
            max_delay_threshold=0.15,
            logger=self.logger
        )
        # World transform timeout & metrics (configurable)
        self.world_tf_timeout = vehicle_config.get('world_tf_timeout', 0.05)  # seconds
        self.world_tf_timeouts = 0
        self.world_tf_errors = 0
        self.world_tf_last_warning = 0.0
        self._prev_time_monotonic = None  # For velocity dt stability

        # Initialize communication handler
        # Use non-blocking mode for process-based vehicles with bidirectional support
        try:
            # Get peer communication configuration for bidirectional communication
            peer_ports = vehicle_config.get('peer_ports', {})
            communication_mode = vehicle_config.get('communication_mode', 'unidirectional')

            self.comm = CommHandler(
                vehicle_id=self.vehicle_id,
                target_ip=self.target_ip,
                send_port=self.send_port,
                recv_port=self.recv_port,
                ack_port=self.ack_port,
                logger=self.comm_logger,
                mode='non_blocking',
                peer_ports=peer_ports,
                communication_mode=communication_mode
            )
            print(f"Vehicle {self.vehicle_id}: Communication initialized in {communication_mode} mode")
        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: Communication initialization failed: {e}")
            self.comm = None

        # Initialize communication in non-blocking mode
        if self.comm is not None:
            self.comm.initialize_sockets()
            print(f"Vehicle {self.vehicle_id}: Communication sockets initialized")
        else:
            print(f"Vehicle {self.vehicle_id}: Warning - Communication not available")

        # Control components based on vehicle role
        try:
            if self.is_leader:
                self.leader_controller = VehicleLeaderController(
                    vehicle_id=self.vehicle_id,
                    config=self.config,
                    logger=self.control_logger
                )
                self.follower_controller = None
                print(f"Vehicle {self.vehicle_id}: Leader controller initialized")
            else:
                self.follower_controller = VehicleFollowerController(
                    vehicle_id=self.vehicle_id,
                    controller_type=self.controller_type,
                    config=self.config,
                    logger=self.control_logger
                )
                self.leader_controller = None
                print(f"Vehicle {self.vehicle_id}: Follower controller ({self.controller_type}) initialized")
        except Exception as e:
            print(f"Vehicle {self.vehicle_id}: Controller initialization failed: {e}")
            self.logger.error(f"Vehicle {self.vehicle_id}: Controller initialization failed: {e}")
            self.leader_controller = None
            self.follower_controller = None
            # Don't stop the event here, let the vehicle continue with basic operation

        # Initialize Vehicle Observer
        spawn_location = vehicle_config.get('spawn_location', [0, 0, 0])
        spawn_rotation = vehicle_config.get('spawn_rotation', [0, 0, 0])
        initial_pose = np.array([spawn_location[0], spawn_location[1], spawn_rotation[2]])
        try:
            self.observer = VehicleObserver(
                vehicle_id=self.vehicle_id,
                fleet_size=self.fleet_size,
                config=self.config,  # Pass proper config
                logger=self.logger,
                initial_pose=initial_pose
            )
        except Exception as e:
            print(f"Vehicle {self.vehicle_id}: Observer initialization failed: {e}")
            self.observer = None

        # Initialize control input tracking
        self.current_control_input = np.array([0.0, 0.0])

        # Sensor data tracking
        self.last_gyroscope_z = 0.0
        self.last_motor_tach = 0.0

        self.gps_data_cache = {
            'position': None,
            'rotation': None,
            'velocity': 0.0,
            'available': False,
            'timestamp': 0.0,
            'last_update': 0.0
        }

        # Leader tracking for followers
        self.leader_vehicle = None
        self.leader_state = None

        print(f"Vehicle {self.vehicle_id} process initialized successfully")
        self.logger.info(f"Vehicle {self.vehicle_id} process initialized successfully")

    def configure_logging_for_performance(self):
        """Configure logging settings for optimal performance."""
        disable_all_logging()
        set_module_logging('observer', True)
        set_module_logging('communication', True)
        set_module_logging('fleet_observer', True)  # Enable fleet observer logging
        
        # Set fleet observer logger to INFO level to see the velocity debug messages
        self.fleet_observer_logger.setLevel(logging.INFO)
        self.comm_logger.setLevel(logging.INFO)
        self.observer_logger.setLevel(logging.INFO)

    def update_gps_data(self):
        """Update GPS data from QCar sensors (supports both virtual and physical QCar modes)."""
        try:
            gps_start_time = time.perf_counter()
            current_time = time.time()
            
            if self.use_physical_qcar and self.physical_qcar is not None:
                # Physical QCar mode with EKF
                self._update_physical_qcar_data(current_time)
            else:
                # Virtual QCar mode (default)
                self._update_virtual_qcar_data(current_time)
                
            gps_duration = (time.perf_counter() - gps_start_time) * 1000
            if gps_duration > 2.0:  # Log if GPS update takes >2ms
                self.gps_logger.warning(f"Vehicle {self.vehicle_id}: GPS update took {gps_duration:.4f}ms")
                
        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: GPS update error: {e}")
            self.gps_data_cache['available'] = False
            print(f"Vehicle {self.vehicle_id}: GPS update exception: {e}")

    def _update_virtual_qcar_data(self, current_time):
        """Update using virtual QCar (QLabs) API."""
        # Get current transform from QCar with timeout protection
        result = self._get_world_transform_with_timeout()
        if result is None:
            # Fallback to last known values if available
            if hasattr(self, '_last_valid_location') and self._last_valid_location is not None:
                location = self._last_valid_location
                rotation = self._last_valid_rotation
                if not hasattr(self, '_timeout_warning_issued'):
                    self.logger.warning(f"Vehicle {self.vehicle_id}: get_world_transform() timeout - using last cached pose")
                    self._timeout_warning_issued = True
            else:
                # Nothing we can do; mark GPS cache unavailable and return early
                self.gps_data_cache['available'] = False
                return
        else:
            try:
                _, location, rotation, _ = result
            except Exception:
                # Defensive: unexpected structure
                self.logger.error(f"Vehicle {self.vehicle_id}: Unexpected get_world_transform() return format: {result}")
                self.gps_data_cache['available'] = False
                return
            # Cache successful result
            self._last_valid_location = location
            self._last_valid_rotation = rotation
            if hasattr(self, '_timeout_warning_issued'):
                # Reset warning flag after a successful call
                delattr(self, '_timeout_warning_issued')
        
        # # Debug: Print GPS data for leader vehicle
        # if self.is_leader and self.vehicle_id == 0:
        #     print(f"Vehicle {self.vehicle_id}: Virtual GPS data: location={location}, rotation={rotation}")
        
        if location is not None and rotation is not None:
            # Calculate velocity
            now_mono = time.monotonic()
            if self._prev_time_monotonic is not None and self.prev_pos is not None:
                dt = now_mono - self._prev_time_monotonic
                if dt > 0:
                    dx = location[0] - self.prev_pos[0]
                    dy = location[1] - self.prev_pos[1]
                    self.velocity = math.sqrt(dx*dx + dy*dy) / dt
                    
                    # # Debug: Print movement info for leader
                    # if self.is_leader and self.vehicle_id == 0:
                    #     print(f"Vehicle {self.vehicle_id}: Virtual movement: velocity={self.velocity:.3f}")
            
            # Update current state
            self.current_pos = list(location)
            self.current_rot = list(rotation)
            self.prev_pos = list(location)
            self.prev_time = current_time
            self._prev_time_monotonic = now_mono
            
            # Update GPS cache
            self.gps_data_cache.update({
                'position': list(location),
                'rotation': list(rotation),
                'velocity': self.velocity,
                'available': True,
                'timestamp': current_time,
                'last_update': current_time
            })
        else:
            self.gps_data_cache['available'] = False
            print(f"Vehicle {self.vehicle_id}: QCar get_world_transform() returned None - using cached position")

    

    def _get_world_transform_with_timeout(self, timeout: float = 0.1):
        """
        Retrieve world transform with a timeout to prevent stalls.
        Uses a per-instance single-thread executor to run the blocking call.

        Args:
            timeout: Maximum seconds to wait for the transform.

        Returns:
            Tuple (id, location, rotation, extra) on success, or None on timeout/error.
        """
        from concurrent.futures import ThreadPoolExecutor, TimeoutError
        eff_timeout = timeout if timeout is not None else getattr(self, 'world_tf_timeout', 0.1)
        try:
            # Lazy-create executor
            if not hasattr(self, '_world_tf_executor') or self._world_tf_executor is None:
                self._world_tf_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix=f"qcar_tf_{self.vehicle_id}")

            future = self._world_tf_executor.submit(self.qcar.get_world_transform)
            return future.result(timeout=eff_timeout)
        except TimeoutError:
            self.world_tf_timeouts += 1
            # Rate-limit warnings (no more than once per 2s)
            now = time.time()
            if now - self.world_tf_last_warning > 2.0:
                self.logger.warning(f"Vehicle {self.vehicle_id}: get_world_transform() timeout after {eff_timeout*1000:.1f}ms (total timeouts={self.world_tf_timeouts})")
                self.world_tf_last_warning = now
            try:
                future.cancel()
            except Exception:
                pass
            return None
        except Exception as e:
            self.world_tf_errors += 1
            self.logger.error(f"Vehicle {self.vehicle_id}: get_world_transform() error ({self.world_tf_errors}): {e}")
            return None

    def _shutdown_world_tf_executor(self):
        """Shutdown the world transform executor to avoid thread leakage."""
        try:
            if hasattr(self, '_world_tf_executor') and self._world_tf_executor is not None:
                self._world_tf_executor.shutdown(wait=False, cancel_futures=True)
                self._world_tf_executor = None
        except Exception:
            pass

    def _update_physical_qcar_data(self, current_time):
        """Update using physical QCar API with EKF (similar to vehicle_control.py)."""
        if self.physical_qcar is None:
            return
            
        # Read from physical QCar sensors
        self.physical_qcar.read()
        
        if self.enable_steering_control and self.gps is not None:
            # GPS and EKF update
            if hasattr(self.gps, 'readGPS') and self.gps.readGPS():
                # GPS data available
                y_gps = np.array([
                    self.gps.position[0],
                    self.gps.position[1], 
                    self.gps.orientation[2]
                ])
                self.current_pos = [y_gps[0], y_gps[1], 0.0]
                self.current_rot = [0.0, 0.0, y_gps[2]]
                
                # Debug: Print physical QCar data for leader
                if self.is_leader and self.vehicle_id == 0:
                    print(f"Vehicle {self.vehicle_id}: Physical GPS data: pos=[{y_gps[0]:.3f}, {y_gps[1]:.3f}], "
                          f"th={y_gps[2]:.3f}, vel={self.velocity:.3f}")
            available = True
            # Update GPS cache
            self.gps_data_cache.update({
                'position': self.current_pos.copy(),
                'rotation': self.current_rot.copy(),
                'available': available,
                'timestamp': current_time,
                'last_update': current_time
            })
        else:
            available = False

        # Update GPS cache
        self.velocity = self.physical_qcar.motorTach
        self.gps_data_cache['velocity'] = self.velocity

        self.gps_data_cache['available'] = available

    def get_cached_gps_data(self):
        """Get cached GPS data."""
        return self.gps_data_cache.copy()
    
    def get_observer_state_direct(self) -> Optional[dict]:
        """
        Get state directly from observer without caching.
        
        Returns:
            Observer state dictionary with essential fields only: position, rotation, velocity
        """
        if self.observer is None:
            return None
            
        try:
            # Get the local state directly from observer (more efficient)
            local_state = self.observer.get_local_state()
            if local_state is not None and len(local_state) >= 4:
                return {
                    'position': [local_state[0], local_state[1], 0.0],  # [x, y, z] format
                    'rotation': [0.0, 0.0, local_state[2]],  # [roll, pitch, yaw] format
                    'velocity': local_state[3]
                }
            else:
                # Fallback to the formatted method if local state is not available
                return self.observer.get_estimated_state_for_control()
        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: Failed to get observer state: {e}")
            return None
    
    # --------------------- Helper / Refactor Methods ---------------------

    def _ensure_fleet_state_store(self):
        """Ensure the fleet state estimates dict exists."""
        if not hasattr(self, 'fleet_state_estimates') or self.fleet_state_estimates is None:
            self.fleet_state_estimates = {}

    def _set_fleet_vehicle_estimate(self, vehicle_idx: int, vehicle_state: np.ndarray, timestamp: float):
        """Refactored helper: store per-vehicle distributed observer estimate from numpy array."""
        self._ensure_fleet_state_store()
        try:
            self.fleet_state_estimates[vehicle_idx] = {
                'position': [float(vehicle_state[0]), float(vehicle_state[1]), 0.0],
                'rotation': [0.0, 0.0, float(vehicle_state[2])],
                'velocity': float(vehicle_state[3]),
                'timestamp': timestamp
            }
        except Exception:
            # Silent fail to avoid impacting real-time loop
            pass

    def _build_fleet_estimates_message(self, timestamp: float) -> dict:
        """Create the fleet estimates message dict (refactored, single authoritative builder)."""
        if not hasattr(self, 'fleet_state_estimates') or not self.fleet_state_estimates:
            return {}
        msg = {
            'msg_type': 'fleet_estimates',
            'sender_id': self.vehicle_id,
            'timestamp': timestamp,
            'fleet_size': len(self.fleet_state_estimates),
            'estimates': {}
        }
        for vid, state in self.fleet_state_estimates.items():
            msg['estimates'][vid] = {
                'pos': state['position'][:2],
                'rot': state['rotation'],
                'vel': state['velocity'],
                'timestamp': state['timestamp']
            }
        return msg

    # ---------------------------------------------------------------------
    def get_best_available_state(self) -> dict:
        """
        Factory method that returns the best available state estimate.
        Priority: Observer EKF > Raw GPS fallback
        
        Returns:
            Best available state estimate with essential fields: position, rotation, velocity
        """
        # Try observer first (check if EKF is initialized through observer attributes)
        if self.observer is not None:
            observer_state = self.get_observer_state_direct()
            if observer_state and hasattr(self.observer, 'ekf_initialized') and self.observer.ekf_initialized:
                return observer_state
        
        # Fallback to raw GPS if observer not available or EKF not initialized
        return {
            'position': self.current_pos.copy(),
            'rotation': self.current_rot.copy(),
            'velocity': self.velocity
        }
        
    def get_state_for_control(self) -> dict:
        """Get current vehicle state for control algorithms using observer estimates."""
        return self.get_best_available_state()
    
    def leader_control_logic(self):
        """Leader control logic that delegates to VehicleLeaderController."""
        if not self.is_leader or self.leader_controller is None:
            return
            
        # Get state for control (uses observer EKF estimates when available)
        control_state = self.get_state_for_control()
        
        # # Debug: Print control state for leader vehicle
        # if self.vehicle_id == 0:  # Assuming leader is vehicle 0
        #     print(f"Vehicle {self.vehicle_id}: Control state for leader: pos={control_state['position']}, "
        #           f"rot={control_state['rotation']}, vel={control_state['velocity']:.3f}, "
        #           f"source={control_state['source']}")
        
        # Calculate actual time elapsed since last control update
        current_time = time.time()
        if not hasattr(self, '_last_control_time'):
            self._last_control_time = current_time
            dt = 1.0 / self.update_rate  # Use nominal dt for first iteration
        else:
            dt = current_time - self._last_control_time
            # Clamp dt to reasonable bounds to avoid instability
            dt = max(0.001, min(dt, 0.1))  # Between 1ms and 100ms
        
        # print("dt" , dt )
        self._last_control_time = current_time
        
        # Compute control commands using the dedicated leader controller
        forward_speed, steering_angle = self.leader_controller.compute_control_auto(
            current_pos=control_state['position'],
            current_rot=control_state['rotation'],
            velocity=control_state['velocity'],
            dt=dt  # Use actual elapsed time
        )

        # # Debug: Print leader control commands
        # if self.vehicle_id == 0:  # Assuming leader is vehicle 0
        #     print(f"Leader control commands: forward={forward_speed:.3f}, steering={steering_angle:.3f}, dt={dt:.3f}")

        # Update control input for observer
        # Convert controller outputs to observer format [steering, acceleration]
        self.current_control_input = np.array([steering_angle, forward_speed])
        
        # Store steering angle for physical QCar EKF
        self._last_steering_angle = steering_angle
        
        # Apply control commands to vehicle based on mode
        # NOTE: Previously, when use_control_observer_mode was True we skipped sending any commands.
        # This caused the leader to remain at its spawn position for virtual (non-physical) runs.
        # We now only suppress direct virtual commands if BOTH observer mode is enabled AND a physical QCar is active.
        
        if  self.use_physical_qcar and self.physical_qcar is not None:
            # Physical QCar mode - use write method like in vehicle_control.py
            self.physical_qcar.write(forward_speed, steering_angle)
            
            # # Debug: Print physical control commands
            # if self.is_leader and self.vehicle_id == 0:
            #     print(f"Vehicle {self.vehicle_id}: Physical control: u={forward_speed:.3f}, delta={steering_angle:.3f}")
        elif not self.use_control_observer_mode:
            # Virtual QCar mode - use QLabs API
            self.qcar.set_velocity_and_request_state(
                forward=forward_speed,
                turn=steering_angle,
                headlights=False,
                leftTurnSignal=False,
                rightTurnSignal=False,
                brakeSignal=False,
                reverseSignal=False
            )
            
            # # Debug: Print virtual control commands  
            # if self.is_leader and self.vehicle_id == 0:
            #     print(f"Vehicle {self.vehicle_id}: Virtual control: forward={forward_speed:.3f}, turn={steering_angle:.3f}")
        
        # Normalize numeric types (convert numpy types to plain Python floats) before broadcasting
        control_state['position'] = [float(x) for x in control_state['position']]
        control_state['rotation'] = [float(r) for r in control_state['rotation']]
        control_state['velocity'] = float(control_state['velocity'])

        # # Extra debug for movement diagnosis
        # self.control_logger.debug(
        #     f"Vehicle {self.vehicle_id}: Cmd fwd={forward_speed:.3f}, steer={steering_angle:.3f}, src={control_state['source']}"
        # )

        # NOTE: State broadcasting is now handled in the observer_update() method
        # This is cleaner since observer maintains the most accurate state estimate
        
        # Log control timing for debugging
        if self.logger.isEnabledFor(logging.DEBUG):
            target_dt = 1.0 / self.update_rate
            timing_error = abs(dt - target_dt) / target_dt * 100
            self.logger.debug(f"Leader control timing - "
                            f"dt={dt:.4f}s (target={target_dt:.4f}s, error={timing_error:.1f}%)")

    def follower_control_logic(self):
        """Follower control logic that delegates to VehicleFollowerController."""
        if self.is_leader or self.follower_controller is None:
            if not hasattr(self, '_follower_skip_logged'):
                print(f"Vehicle {self.vehicle_id}: Skipping follower control - is_leader={self.is_leader}, controller_exists={self.follower_controller is not None}")
                self._follower_skip_logged = True
            return
        
        # print(f"Vehicle {self.vehicle_id}: Executing follower control logic")
        try:
            # Use GPS-synchronized time for better prediction
            current_gps_time = self.gps_sync.get_synced_time()
            self.comm_logger.debug(f"Vehicle {self.vehicle_id}: Follower control - GPS time: {current_gps_time}")
            
            # NEW: Get the state data from the target vehicle (chain-following)
            target_data = None
            
            # Try to get interpolated state for current time (most accurate)
            interpolated_state = self.get_interpolated_target_state(current_gps_time)
            self.comm_logger.debug(f"Vehicle {self.vehicle_id}: Interpolated state from target vehicle {self.following_target}: {interpolated_state}")
            if interpolated_state:
                target_data = {
                    'position': interpolated_state.get('pos', [0, 0, 0]),
                    'rotation': interpolated_state.get('rot', [0, 0, 0]),
                    'velocity': interpolated_state.get('v', 0.0),
                    'timestamp': interpolated_state.get('timestamp', current_gps_time),
                    'interpolated': interpolated_state.get('interpolated', False),
                    'processing_delay': interpolated_state.get('processing_delay', 0.0)
                }
                self.logger.debug(f"Using interpolated target data from vehicle {self.following_target}: pos={target_data['position']}, "
                                f"v={target_data['velocity']:.3f}, interpolated={target_data['interpolated']}")
            
            # Fallback: use latest valid state if interpolation not available
            elif self.leader_state is not None:
                self.comm_logger.info(f"Vehicle {self.vehicle_id}: Using fallback leader state (backward compatibility): {self.leader_state}")
                # Validate that the leader_state is recent enough
                state_age = current_gps_time - self.leader_state.get('timestamp', 0)
                if state_age <= 1.0:  # Use state if less than 1 second old
                    target_data = {
                        'position': self.leader_state.get('pos', [0, 0, 0]),
                        'rotation': self.leader_state.get('rot', [0, 0, 0]),
                        'velocity': self.leader_state.get('v', 0.0),
                        'timestamp': self.leader_state.get('timestamp', current_gps_time),
                        'interpolated': False,
                        'state_age': state_age
                    }
                    self.logger.debug(f"Using fallback leader data: age={state_age:.3f}s")
                else:
                    self.comm_logger.warning(f"Target state too old ({state_age:.3f}s), stopping vehicle")
                    # Stop vehicle if target data is too old
                    self.qcar.set_velocity_and_request_state(forward=0.0, turn=0.0, headlights=False,
                                                        leftTurnSignal=False,
                                                        rightTurnSignal=False,
                                                        brakeSignal=False,
                                                        reverseSignal=False)
                    self.current_control_input = np.array([0.0, 0.0])
                    return
            
            # No target data available at all
            if target_data is None:
                self.comm_logger.error(f"Vehicle {self.vehicle_id}: No target data available from vehicle {self.following_target}, stopping vehicle")
                self.qcar.set_velocity_and_request_state(forward=0.0, turn=0.0, headlights=False,
                                                        leftTurnSignal=False,
                                                        rightTurnSignal=False,
                                                        brakeSignal=False,
                                                        reverseSignal=False)
                self.current_control_input = np.array([0.0, 0.0])
                return
            
            # Get current vehicle state for control
            current_state = self.get_state_for_control()
            
            # Compute control using the follower controller
            # NEW: Use target_data (from the vehicle this one should follow) instead of leader_data
            forward_speed, steering_angle = self.follower_controller.compute_control(
                current_pos=current_state['position'],
                current_rot=current_state['rotation'],
                current_velocity=current_state['velocity'],
                leader_pos=target_data['position'],  # Position of the target vehicle to follow
                leader_rot=target_data['rotation'],  # Rotation of the target vehicle to follow
                leader_velocity=target_data['velocity'],  # Velocity of the target vehicle to follow
                leader_timestamp=target_data['timestamp'],
                dt=1.0 / self.update_rate
            )
            
            # print(f"Vehicle {self.vehicle_id}: Following vehicle {self.following_target} - "
            #       f"Target pos: {target_data['position']}, My pos: {current_state['position']}")
            
            # Update control input for observer
            self.current_control_input = np.array([steering_angle, forward_speed])
            
            # Apply control commands to vehicle
            self.qcar.set_velocity_and_request_state(
                forward=forward_speed,
                turn=steering_angle,
                headlights=False,
                leftTurnSignal=False,
                rightTurnSignal=False,
                brakeSignal=False,
                reverseSignal=False
            )

            # NOTE: State broadcasting is now handled in the observer_update() method
            # This is cleaner since observer maintains the most accurate state estimate

            # Debug logging (updated to show chain-following)
            if self.logger.isEnabledFor(logging.DEBUG):
                self.logger.debug(f"Follower control: forward={forward_speed:.3f}, steering={steering_angle:.3f}, "
                                f"target_vehicle={self.following_target}, target_pos={target_data['position']}, current_pos={current_state['position']}")
                
        except Exception as e:
            self.comm_logger.error(f"Vehicle {self.vehicle_id}: Follower control error: {e}")
            # Reset control input on error
            self.current_control_input = np.array([0.0, 0.0])
            self.qcar.set_velocity_and_request_state(forward=0.0, turn=0.0, headlights=False,
                leftTurnSignal=False,
                rightTurnSignal=False,
                brakeSignal=False,
                reverseSignal=False)

    def get_interpolated_leader_state(self, target_time: Optional[float] = None) -> Optional[dict]:
        """Get interpolated leader state from state queue."""
        if target_time is None:
            target_time = time.time()
        
        # Add debug info about queue state
        queue_stats = self.state_queue.get_queue_stats()
        self.comm_logger.debug(f"Vehicle {self.vehicle_id}: Queue stats - size: {queue_stats['current_queue_size']}, "
              f"total_received: {queue_stats['total_received']}, valid: {queue_stats['valid_states']}")
        
        # Get all states to see what's in the queue
        all_states = self.state_queue.get_all_states(sender_id=0)  # Leader is vehicle 0
        self.comm_logger.debug(f"Vehicle {self.vehicle_id}: States from leader in queue: {len(all_states)}")
        
        result = self.state_queue.get_interpolated_state(target_time, sender_id=0)
        self.comm_logger.debug(f"Vehicle {self.vehicle_id}: Interpolated result: {result}")
        return result

    def get_interpolated_target_state(self, target_time: Optional[float] = None) -> Optional[dict]:
        """
        NEW: Get interpolated state from the target vehicle this one should follow.
        For chain-following: Vehicle 1 follows 0, Vehicle 2 follows 1, Vehicle 3 follows 2, etc.
        """
        if target_time is None:
            target_time = time.time()
        
        # Determine which vehicle to get state from
        if self.following_target is None:
            # This is a leader, no target to follow
            return None
        
        target_vehicle_id = self.following_target
        
        # Add debug info about queue state for the target vehicle
        queue_stats = self.state_queue.get_queue_stats()
        self.comm_logger.debug(f"Vehicle {self.vehicle_id}: Looking for target vehicle {target_vehicle_id} - Queue stats - size: {queue_stats['current_queue_size']}")
        
        # Get all states from the target vehicle
        all_states = self.state_queue.get_all_states(sender_id=target_vehicle_id)
        self.comm_logger.debug(f"Vehicle {self.vehicle_id}: States from target vehicle {target_vehicle_id} in queue: {len(all_states)}")
        
        # Get interpolated state from the target vehicle
        result = self.state_queue.get_interpolated_state(target_time, sender_id=target_vehicle_id)
        self.comm_logger.debug(f"Vehicle {self.vehicle_id}: Interpolated result from vehicle {target_vehicle_id}: {result}")
        return result

    def run(self):
        """Main run loop for the vehicle process."""
        self.logger.info(f"Vehicle {self.vehicle_id}: Starting main run loop")
        
        # Initialize timing
        control_dt = 1.0 / self.update_rate
        observer_dt = 1.0 / self.observer_rate
        gps_dt = 1.0 / self.gps_update_rate
        
        last_control_time = time.time()
        last_observer_time = time.time()
        last_gps_time = time.time()
        last_status_time = time.time()
        

        
        try:
            while self.running.is_set() and not self.stop_event.is_set():
                current_time = time.time()
                # print(f"Vehicle {self.vehicle_id}: Main loop iteration at {current_time:.3f}")
                # GPS data collection
                if current_time - last_gps_time >= gps_dt:
                    self.update_gps_data()
                    last_gps_time = current_time
                
                # Observer update
                if current_time - last_observer_time >= observer_dt:
                    self.observer_update()
                    last_observer_time = current_time
                
                # Control loop
                if current_time - last_control_time >= control_dt:
                    if self.is_leader:
                        self.leader_control_logic()
                        # print(f"Vehicle {self.vehicle_id}: Leader control logic executed")
                    else:
                        # print(f"Vehicle {self.vehicle_id}: Follower control logic executed")
                        self.follower_control_logic()
                    last_control_time = current_time
                
                # Handle communication
                self.handle_communication()
                
                # Send status update periodically
                if current_time - last_status_time >= 5.0:
                    self.send_status_update()
                    last_status_time = current_time
                
                # Small sleep to prevent excessive CPU usage
                time.sleep(0.001)  # 1ms
                
        except Exception as e:
            print(f"Vehicle {self.vehicle_id}: Main loop error: {e}")
            # self.logger.error(f"Vehicle {self.vehicle_id}: Main loop error: {e}")
        finally:
            self.cleanup()

    def stop(self):
        """Gracefully stop the vehicle process control logic."""
        self.logger.info(f"Vehicle {self.vehicle_id}: Stopping control logic")
        
        # First stop the running flag to exit the main loop
        self.running.clear()
        
        # Stop all control immediately with zero commands
        try:
            if self.use_physical_qcar and self.physical_qcar is not None:
                # Stop physical QCar
                self.physical_qcar.write(0, 0)
                print(f"Vehicle {self.vehicle_id}: Physical QCar stopped with zero commands")
            else:
                # Stop virtual QCar
                self.qcar.set_velocity_and_request_state(
                        forward=0.0, 
                        turn=0.0,
                        headlights=False,
                        leftTurnSignal=False,
                        rightTurnSignal=False,
                        brakeSignal=False,
                        reverseSignal=False
                    )
                print(f"Vehicle {self.vehicle_id}: Virtual QCar stopped with zero commands")
        except Exception as e:
            print(f"Vehicle {self.vehicle_id}: Error stopping vehicle: {e}")
        
        # Close QLabs connection
        try:
            if hasattr(self, 'qlabs') and self.qlabs is not None:
                self.qlabs.close()
                print(f"Vehicle {self.vehicle_id}: QLabs connection closed")
        except Exception as e:
            print(f"Vehicle {self.vehicle_id}: Error closing QLabs: {e}")
            
        self.logger.info(f"Vehicle {self.vehicle_id}: Control logic stopped")

    def observer_update(self):
        """
        Update observer with current GPS data, sensor data, and control inputs.
        Similar to Vehicle.py observer_loop but simplified for process context.
        """
        if self.observer is None:
            return
            
        try:
            current_time = self.gps_sync.get_synced_time()
            
            # Get GPS data
            gps_data = self.get_cached_gps_data()
            measured_state = None
            if gps_data['available']:
                # Convert GPS data to numpy array format expected by observer
                # Observer expects [x, y, theta, v] format
                measured_state = np.array([
                    gps_data['position'][0],  # x
                    gps_data['position'][1],  # y
                    gps_data['rotation'][2],  # theta (yaw angle)
                    gps_data['velocity']      # velocity
                ])
            # TODO : Should use gyroscope data if we use physic RT model 
            motor_tach = self.velocity  # Fallback to velocity estimate
            gyroscope_z = 0  # Always numeric to satisfy logging formatting

            
            # -------- Update observer local state
            estimated_state = self.observer.update_local_state(
                measured_state=measured_state,
                control_input=self.current_control_input,
                timestamp=current_time,
                motor_tach=motor_tach,
                gyroscope_z=gyroscope_z
            )

            # -------- Update distributed observer with received states from other vehicles
            try:
                # Get distributed fleet state estimates
                fleet_states = self.observer.update_distributed_estimates(self.current_control_input ,estimated_state, current_time )
                
                # Log distributed observer status
                if fleet_states is not None:
                    self.observer_logger.debug(f"Vehicle {self.vehicle_id}: Distributed observer updated - "
                                             f"Fleet size: {fleet_states.shape[1]}, "
                                             f"State dim: {fleet_states.shape[0]}")
                    
                    # Update fleet state estimates cache using helper
                    for vehicle_idx in range(fleet_states.shape[1]):
                        vehicle_state = fleet_states[:, vehicle_idx]
                        self._set_fleet_vehicle_estimate(vehicle_idx, vehicle_state, current_time)
                        if vehicle_idx != self.vehicle_id:
                            self.observer_logger.debug(
                                f"Vehicle {self.vehicle_id}: Estimated vehicle {vehicle_idx} - "
                                f"pos=({vehicle_state[0]:.3f}, {vehicle_state[1]:.3f}), vel={vehicle_state[3]:.3f}")
                
            except Exception as dist_error:
                self.observer_logger.warning(f"Vehicle {self.vehicle_id}: Distributed observer error: {dist_error}")
            
            # Use the returned estimated_state directly instead of making redundant call
            if estimated_state is not None and len(estimated_state) >= 4:
                # Convert numpy array [x, y, theta, v] to our cache format
                # Debug log to verify observer update
                self.observer_logger.debug(f"Observer updated: pos=({estimated_state[0]:.3f}, {estimated_state[1]:.3f}), "
                                         f"vel={estimated_state[3]:.3f}, "
                                         f"ekf_init={getattr(self.observer, 'ekf_initialized', False)}")
                
                # NEW: Broadcast this vehicle's updated state to all other vehicles
                # This is the logical place since observer maintains the most accurate state estimate
                self.broadcast_own_state()
                
                # NEW: Broadcast fleet estimates if distributed observer is working
                if hasattr(self, 'fleet_state_estimates') and len(self.fleet_state_estimates) > 1:
                    self.broadcast_fleet_estimates()
                
            else:
                self.observer_logger.warning(f"Vehicle {self.vehicle_id}: Invalid estimated state from observer: {estimated_state}")
                
        except Exception as e:
            self.observer_logger.error(f"Vehicle {self.vehicle_id}: Observer update error: {e}")

    def broadcast_fleet_estimates(self):
        """
        Broadcast fleet state estimates from distributed observer.
        This sends the global view of all vehicle states.
        """
        if self.comm is None or not hasattr(self, 'fleet_state_estimates'):
            return
            
        try:
            current_time = self.gps_sync.get_synced_time()
            
            # Build message using helper
            fleet_message = self._build_fleet_estimates_message(current_time)
            if not fleet_message:
                return
            
            
            # Broadcast fleet estimates to all vehicles
            success = self.comm.send_fleet_estimates_broadcast(fleet_message)
            
            if success:
                self.observer_logger.debug(f"Vehicle {self.vehicle_id}: Broadcasted fleet estimates for {len(self.fleet_state_estimates)} vehicles")
            else:
                self.observer_logger.warning(f"Vehicle {self.vehicle_id}: Failed to broadcast fleet estimates")
                
        except Exception as e:
            self.observer_logger.error(f"Vehicle {self.vehicle_id}: Fleet broadcast error: {e}")

    def process_received_fleet_estimates(self, fleet_message: dict):
        """
        Process received fleet estimates from another vehicle's distributed observer.
        
        Args:
            fleet_message: Dictionary containing fleet estimates from another vehicle
        """
        try:
            sender_id = fleet_message.get('sender_id')
            fleet_size = fleet_message.get('fleet_size', 0)
            estimates = fleet_message.get('estimates', {})
            
            if sender_id == self.vehicle_id:
                return  # Ignore our own messages
            
            # Log the received fleet estimates using fleet logger
            if hasattr(self, 'fleet_logger'):
                self.fleet_logger.info(f"RECEIVED_FLEET_ESTIMATES: From Vehicle {sender_id}, "
                                     f"Fleet size: {fleet_size}, Estimates count: {len(estimates)}")
                
                # Log detailed estimates for debugging
                for vehicle_id, state in estimates.items():
                    pos = state.get('pos', [0, 0])
                    vel = state.get('vel', 0.0)
                    self.fleet_logger.debug(f"FLEET_EST_V{vehicle_id}: pos=({pos[0]:.3f},{pos[1]:.3f}), "
                                          f"vel={vel:.3f}")
            else:
                self.observer_logger.info(f"RECEIVED_FLEET_ESTIMATES: From Vehicle {sender_id}, "
                                        f"Fleet size: {fleet_size}, Estimates count: {len(estimates)}")
            
            # Optional: Update our own distributed observer with received estimates
            # This could be used for consensus or validation
            if hasattr(self, 'observer') and hasattr(self.observer, 'update_from_peer_estimates'):
                self.observer.update_from_peer_estimates(sender_id, estimates)
                
        except Exception as e:
            self.observer_logger.error(f"Vehicle {self.vehicle_id}: Error processing received fleet estimates: {e}")

    def broadcast_own_state(self):
        """
        Broadcast this vehicle's current state to all other vehicles in the fleet.
        This method provides bidirectional communication where every vehicle shares its state.
        """
        if self.comm is None:
            return
            
        try:
            # Get the current best available state (simplified)
            current_state = self.get_best_available_state()
            
            # Add only necessary fields for network transmission
            broadcast_state = {
                'vehicle_id': self.vehicle_id,
                'position': [float(x) for x in current_state['position']],
                'rotation': [float(r) for r in current_state['rotation']],
                'velocity': float(current_state['velocity']),
                'timestamp': time.time(),
                'control_input': [float(x) for x in self.current_control_input]
            }
            
            # Broadcast state to all other vehicles
            self.comm.send_state_broadcast(broadcast_state)
            
            # Enhanced debug logging
            self.comm_logger.debug(
                f"Vehicle {self.vehicle_id}: Broadcasted state - pos={broadcast_state['position']}, "
                f"vel={broadcast_state['velocity']:.3f}, control={broadcast_state['control_input']}"
            )
            
        except Exception as e:
            self.comm_logger.error(f"Vehicle {self.vehicle_id}: Failed to broadcast own state: {e}")

    def handle_communication(self):
        """
        Handle non-blocking communication with direct processing.
        
        FIXED: Removed the intermediate process_received_state() method that was causing
        data loss and unnecessary complexity. Now all communication data is processed
        directly without additional conversions or method calls that could lose data.
        
        This ensures that the exact data received in handle_communication() is preserved
        and processed correctly for both vehicle states and fleet estimates.
        """
        try:
            # Check for incoming messages
            if self.comm is not None:
                received_data = self.comm.receive_state_non_blocking()
                if received_data:
                    # Process data directly without unnecessary intermediate method
                    self._process_communication_data_direct(received_data)
                else:
                    # Only log this occasionally to avoid spam
                    if not hasattr(self, '_last_no_data_log') or time.time() - self._last_no_data_log > 1.0:
                        self.comm_logger.debug(f"Vehicle {self.vehicle_id}: No data received from peers")
                        self._last_no_data_log = time.time()
            else:
                # Log communication handler not available
                if not hasattr(self, '_last_comm_error_log') or time.time() - self._last_comm_error_log > 10.0:
                    self.comm_logger.error(f"Vehicle {self.vehicle_id}: Communication handler not available")
                    self._last_comm_error_log = time.time()
                
        except Exception as e:
            self.comm_logger.error(f"Vehicle {self.vehicle_id}: Communication error: {e}")
            print(f"Vehicle {self.vehicle_id}: Communication error: {e}")  # Keep print for critical errors

    def _process_communication_data_direct(self, received_data: dict):
        """Process received communication data directly without data loss."""
        try:
            # Handle different message types based on 'type' or 'msg_type' field
            msg_type = received_data.get('type', received_data.get('msg_type', 'vehicle_state'))
            
            if msg_type == 'fleet_estimates':
                # Process fleet estimates from distributed observer
                self._handle_fleet_estimates_direct(received_data)
            else:
                # Process individual vehicle state directly
                self._handle_vehicle_state_direct(received_data)
                
        except Exception as e:
            self.comm_logger.error(f"Vehicle {self.vehicle_id}: Error processing communication data: {e}")

    def _handle_fleet_estimates_direct(self, fleet_message: dict):
        """Handle fleet estimates directly without data conversion loss."""
        try:
            sender_id = fleet_message.get('sender_id', fleet_message.get('vehicle_id', -1))
            estimates = fleet_message.get('estimates', {})
            message_timestamp = fleet_message.get('timestamp', time.time())
            seq = fleet_message.get('seq', -1)
            
            # Log fleet estimates with complete info but concise format
            est_data = {}
            for veh_id, est in estimates.items():
                pos = est.get('pos', [0, 0])
                rot = est.get('rot', [0, 0, 0])
                vel = est.get('vel', 0.0)
                est_data[f"V{veh_id}"] = f"Pos=({pos[0]:.4f},{pos[1]:.4f}) Rot={rot[2]:.4f} Vel={vel:.4f}"
            
            self.fleet_observer_logger.info(f"RECV_FLEET From=V{sender_id} Seq={seq} T={message_timestamp:.3f} {est_data}")
            
            # Update our own fleet estimates with external observations
            if not hasattr(self, 'external_fleet_estimates'):
                self.external_fleet_estimates = {}
            
            self.external_fleet_estimates[sender_id] = {
                'timestamp': message_timestamp,
                'estimates': estimates
            }
            
            # Add fleet estimates to observer if available
            if self.observer is not None:
                success = self.observer.add_received_state_fleet(
                    sender_id=sender_id,
                    fleet_estimates=estimates,
                    timestamp=message_timestamp
                )
                if success:
                    self.observer_logger.info(f"Vehicle {self.vehicle_id}: Added fleet estimates to observer - "
                                             f"sender is ={sender_id}")
            else:
                self.observer_logger.warning(f"Vehicle {self.vehicle_id}: Observer not initialized for fleet estimates")

        except Exception as e:
            self.observer_logger.error(f"Vehicle {self.vehicle_id}: Error processing fleet estimates: {e}")

    def _handle_vehicle_state_direct(self, received_state: dict):
        """Handle individual vehicle state directly without data conversion loss."""
        try:
            # Extract sender information
            sender_id = received_state.get('vehicle_id', received_state.get('id', received_state.get('sender_id', -1)))
            seq = received_state.get('seq', -1)
            timestamp = received_state.get('timestamp', time.time())
            
            # Extract state data directly without conversion
            pos = received_state.get('pos', received_state.get('position', [0, 0, 0]))
            rot = received_state.get('rot', received_state.get('rotation', [0, 0, 0]))
            vel = received_state.get('v', received_state.get('vel', received_state.get('velocity', 0.0)))
            control = received_state.get('ctrl_u', received_state.get('control_input', [0.0, 0.0]))
            
            
            #------ Log with all the original data preserved
            self.comm_logger.info(f"STATE_RECV From=V{sender_id} Seq={seq} T={timestamp:.3f} "
                                f"Pos=({pos[0]:.4f},{pos[1]:.4f}) Rot={rot[2]:.4f} Vel={vel:.4f} "
                                f"Control=({control[0]:.3f},{control[1]:.3f})")
            
            # Add to state queue with original data structure preserved
            success = self.state_queue.add_state(received_state, self.gps_sync)
            
            if not success:
                self.comm_logger.warning(f"Vehicle {self.vehicle_id}: State rejected by queue! "
                                       f"Data: sender={sender_id}, seq={seq}, timestamp={timestamp}")
                # Log the exact data that was rejected
                self.comm_logger.warning(f"Vehicle {self.vehicle_id}: REJECTED_DATA_DETAILS: {received_state}")
            else:
                queue_stats = self.state_queue.get_queue_stats()
                self.comm_logger.debug(f"Vehicle {self.vehicle_id}: State added to queue successfully. "
                                     f"Queue size: {queue_stats['current_queue_size']}")
                
                # Update leader_state for fallback (preserve original structure)
                if 'pos' in received_state:
                    self.leader_state = received_state.copy()  # Make a copy to preserve original
                    self.comm_logger.debug(f"Vehicle {self.vehicle_id}: Updated leader_state fallback")

                # Feed received state to distributed observer to use (if available)
                if self.observer is not None and sender_id >= 0:
                    try:
                        # Convert to observer format but preserve all original data
                        if len(pos) >= 2:
                            state_array = np.array([
                                float(pos[0]),  # x
                                float(pos[1]),  # y  
                                float(rot[2]) if len(rot) > 2 else 0.0,  # theta (yaw)
                                float(vel)      # velocity
                            ])
                            
                            # Control input (ensure exactly 2 elements)
                            control_array = np.array([float(control[0]), float(control[1])]) if len(control) >= 2 else np.array([0.0, 0.0])
                            
                            # Add to observer with original timestamp
                            self.observer.add_received_state(
                                sender_id=sender_id,
                                state=state_array,
                                control=control_array,
                                timestamp=timestamp
                            )
                            #--------------- Log: Confirm observer received the correct data

                            self.observer_logger.info(f"Vehicle {self.vehicle_id}: Added state from vehicle {sender_id} to observer - "
                                                     f"pos=({pos[0]:.3f}, {pos[1]:.3f}), vel={vel:.3f}, control={control}")
                        else:
                            self.observer_logger.warning(f"Vehicle {self.vehicle_id}: Invalid position data from vehicle {sender_id}: {pos}")
                        
                    except Exception as obs_error:
                        self.observer_logger.warning(f"Vehicle {self.vehicle_id}: Error feeding state to observer: {obs_error}")
                
        except Exception as e:
            self.comm_logger.error(f"Vehicle {self.vehicle_id}: Error handling vehicle state: {e}")
            # Log the full received_state for debugging
            self.comm_logger.error(f"Vehicle {self.vehicle_id}: ERROR_DATA_DUMP: {received_state}")

    def send_status_update(self):
        """Send status update to main process."""
        try:
            status = {
                'vehicle_id': self.vehicle_id,
                'status': 'running',
                'position': self.current_pos,
                'velocity': self.velocity,
                'timestamp': time.time()
            }
            self.status_queue.put(status)
        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: Status update error: {e}")

    def cleanup(self):
        """Clean up resources before shutting down."""
        try:
            self.logger.info(f"Vehicle {self.vehicle_id}: Cleaning up resources")
            
            # CRITICAL: Stop the vehicle immediately with zero commands
            try:
                if self.use_control_observer_mode and self.vehicle_id == 0 :
                    print(f"Vehicle {self.vehicle_id}: Emergency stop - Physical QCar zero commands sent (control observer mode)")
                elif self.use_physical_qcar and self.physical_qcar is not None:
                    # Stop physical QCar immediately
                    self.physical_qcar.write(0, 0)
                    print(f"Vehicle {self.vehicle_id}: Emergency stop - Physical QCar zero commands sent")
                else:
                    # Stop virtual QCar immediately
                    self.qcar.set_velocity_and_request_state(
                        forward=0.0, 
                        turn=0.0,
                        headlights=False,
                        leftTurnSignal=False,
                        rightTurnSignal=False,
                        brakeSignal=False,
                        reverseSignal=False
                    )
                    print(f"Vehicle {self.vehicle_id}: Emergency stop - Virtual QCar zero commands sent")
            except Exception as e:
                print(f"Vehicle {self.vehicle_id}: Error during emergency stop: {e}")
            
            # Clean up communication
            if hasattr(self, 'comm') and self.comm is not None:
                self.comm.cleanup()
            
            # Clean up controllers safely
            if self.leader_controller is not None:
                self.leader_controller.stop_control()
            if self.follower_controller is not None:
                self.follower_controller.stop_control()

            # # Clean up GPS sync
            # if hasattr(self, 'gps_sync') and self.gps_sync is not None:
            #     self.gps_sync.cleanup()
                
            # Clean up physical QCar resources
            if self.use_physical_qcar:
                if self.physical_qcar is not None:
                    # Close physical QCar connection
                    try:
                        if hasattr(self.physical_qcar, 'close'):
                            self.physical_qcar.close()
                    except:
                        pass  # Ignore errors during cleanup
                if self.gps is not None and hasattr(self.gps, 'close'):
                    # Close GPS connection
                    try:
                        self.gps.close()
                    except:
                        pass  # Ignore errors during cleanup

            # Shutdown transform executor to avoid thread leakage
            self._shutdown_world_tf_executor()
                        
        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: Cleanup error: {e}")
            print(f"Vehicle {self.vehicle_id}: Cleanup error: {e}")
