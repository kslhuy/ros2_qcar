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
    get_control_logger, get_observer_logger, disable_all_logging, set_module_logging
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
    fleet_size = vehicle_config['fleet_size']
    initial_pose = vehicle_config['initial_pose']
    
    # Communication config
    target_ip = vehicle_config['target_ip']
    send_port = vehicle_config['send_port']
    recv_port = vehicle_config['recv_port']
    ack_port = vehicle_config['ack_port']
    
    # GPS config
    gps_server_ip = vehicle_config['gps_server_ip']
    gps_server_port = vehicle_config['gps_server_port']
    
    # Control parameters
    max_steering = vehicle_config.get('max_steering', 0.6)
    lookahead_distance = vehicle_config.get('lookahead_distance', 7.0)
    update_rate = vehicle_config.get('update_rate', 100)
    observer_rate = vehicle_config.get('observer_rate', 100)
    gps_update_rate = vehicle_config.get('gps_update_rate', 50)
    
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

            print(f"Vehicle {vehicle_id}: Spawn success: {success}")
            print(f"Vehicle {vehicle_id}: Spawn parameters - location: {spawn_location}, rotation: {spawn_rotation}, scale: {vehicle_scale}")
            
            if not success:
                print(f"Vehicle {vehicle_id}: Failed to spawn vehicle - spawn_id returned False")
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
                    vehicle.qcar.set_velocity_and_request_state(forward=0.0, turn=0.0)
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
        self.use_physical_qcar = is_leader and PHYSICAL_QCAR_AVAILABLE
        print(f"Vehicle {vehicle_id}: use_physical_qcar={self.use_physical_qcar}, PHYSICAL_QCAR_AVAILABLE={PHYSICAL_QCAR_AVAILABLE}, is_leader={is_leader}")
        self.enable_steering_control = vehicle_config.get('enable_steering_control', True)
        self.calibrate = vehicle_config.get('calibrate', False)
        
        # Initialize QCar interfaces based on mode
        self.physical_qcar = None
        self.ekf = None
        self.gps = None
        
        # if self.use_physical_qcar:
        #     try:
        #         self.rtModel = os.path.normpath(os.path.join(os.environ['RTMODELS_DIR'], 'QCar2/QCar2_Workspace_studio'))
        #         QLabsRealTime().start_real_time_model(self.rtModel, actorNumber=vehicle_id)

        #         # Initialize physical QCar interface for leader
        #         self.physical_qcar = QCar(readMode=1, frequency=self.update_rate)
                
        #         # Initialize EKF and GPS if steering control is enabled
        #         if self.enable_steering_control or self.calibrate:
        #             # Get initial pose from config
        #             initial_pose = vehicle_config.get('initial_pose', [0, 0, 0])
        #             calibration_pose = vehicle_config.get('calibration_pose', [0, 2, -np.pi/2])
                    
        #             self.ekf = QCarEKF(x_0=initial_pose)
        #             self.gps = QCarGPS(initialPose=calibration_pose, calibrate=self.calibrate)
        #             print(f"Vehicle {self.vehicle_id}: Initialized physical QCar with EKF and GPS")
        #         else:
        #             self.gps = memoryview(b'')
        #             print(f"Vehicle {self.vehicle_id}: Initialized physical QCar without GPS")
                    
        #     except Exception as e:
        #         print(f"Vehicle {self.vehicle_id}: Failed to initialize physical QCar: {e}")
        #         self.use_physical_qcar = False
        #         self.physical_qcar = None
        #         self.ekf = None
        #         self.gps = None

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
        
        # Configure logging for performance
        self.configure_logging_for_performance()
        
        # GPS Time Synchronization
        gps_server_ip = vehicle_config.get('gps_server_ip', '127.0.0.1')
        gps_server_port = vehicle_config.get('gps_server_port', 8001)
        self.gps_sync = GPSSync(gps_server_ip, gps_server_port, vehicle_id)
        self.sync_interval = 5.0
        self.last_sync_attempt = time.time()
        
        # State Queue for managing received states
        self.state_queue = StateQueue(
            max_queue_size=50,
            max_age_seconds=2.0,
            max_delay_threshold=1.0,
            logger=self.logger
        )
        
        # Initialize communication handler
        # Use non-blocking mode for process-based vehicles
        try:
            self.comm = CommHandler(
                vehicle_id=self.vehicle_id,
                target_ip=self.target_ip,
                send_port=self.send_port,
                recv_port=self.recv_port,
                ack_port=self.ack_port,
                logger=self.comm_logger,
            mode='non_blocking'
        )
        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: Communication initialization failed: {e}")

        # Initialize communication in non-blocking mode
        self.comm.initialize_sockets()
        

        
        # Control components based on vehicle role
        try:
            if self.is_leader:
                self.leader_controller = VehicleLeaderController(
                    vehicle_id=self.vehicle_id,
                    config=self.config,
                    logger=self.control_logger
                )
                self.follower_controller = None
            else:
                self.follower_controller = VehicleFollowerController(
                    vehicle_id=self.vehicle_id,
                    controller_type=self.controller_type,
                    config=self.config,
                    logger=self.control_logger
                )
                self.leader_controller = None
        except Exception as e:
            print(f"Vehicle {self.vehicle_id}: Controller initialization failed: {e}")
            # self.logger.warning(f"Vehicle {vehicle_id}: Controller initialization failed: {e}")
            self.leader_controller = None
            self.follower_controller = None
            # Don't stop the event here, let the vehicle continue with basic operation
            # stop_event.set()

        # Initialize Vehicle Observer
        try:
            # Configure observer to use EKF (like vehicle_control2.py)
            observer_config = {
                "local_observer_type": "kalman",  # Use EKF instead of direct
                "enable_distributed": True,
                "dt": 1.0 / self.observer_rate
            }
            
            self.observer = VehicleObserver(
                vehicle_id=self.vehicle_id,
                fleet_size=self.fleet_size,
                config=observer_config,  # Pass proper config
                logger=self.logger
            )
        except Exception as e:
            print(f"Vehicle {self.vehicle_id}: Observer initialization failed: {e}")
            self.observer = None
        
        # Initialize EKF in observer with correct 3D pose
        initial_pose = vehicle_config.get('initial_pose')
        if initial_pose is not None and len(initial_pose) >= 3:
            initial_ekf_pose = np.array(initial_pose[:3])  # Only [x, y, theta]
        else:
            # Use spawn location and rotation for initial pose
            spawn_location = vehicle_config.get('spawn_location', [0, 0, 0])
            spawn_rotation = vehicle_config.get('spawn_rotation', [0, 0, 0])
            initial_ekf_pose = np.array([spawn_location[0], spawn_location[1], spawn_rotation[2]])
        
        if self.observer is not None:
            try:
                self.observer.initialize_ekf(initial_ekf_pose)
                print(f"Vehicle {self.vehicle_id}: EKF initialized with pose {initial_ekf_pose}")
            except Exception as e:
                print(f"Vehicle {self.vehicle_id}: EKF initialization failed: {e}")

        # Initialize control input tracking
        self.current_control_input = np.array([0.0, 0.0])
        
        # State caches
        self.observer_state_cache = {
            'position': [0.0, 0.0, 0.0],
            'rotation': [0.0, 0.0, 0.0],
            'velocity': 0.0,
            'gps_available': False,
            'ekf_initialized': False,
            'source': 'unknown',
            'timestamp': 0.0
        }
        
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
        # set_module_logging('timing', True)
        
        # Set communication logger to INFO level to see the debug messages
        self.comm_logger.setLevel(logging.INFO)
        self.observer_logger.setLevel(logging.WARNING)
        
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
            if gps_duration > 5.0:  # Log if GPS update takes >5ms
                self.gps_logger.warning(f"Vehicle {self.vehicle_id}: GPS update took {gps_duration:.2f}ms")
                
        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: GPS update error: {e}")
            self.gps_data_cache['available'] = False
            print(f"Vehicle {self.vehicle_id}: GPS update exception: {e}")

    def _update_virtual_qcar_data(self, current_time):
        """Update using virtual QCar (QLabs) API."""
        # Get current transform from QCar
        _, location, rotation, _ = self.qcar.get_world_transform()
        
        # # Debug: Print GPS data for leader vehicle
        # if self.is_leader and self.vehicle_id == 0:
        #     print(f"Vehicle {self.vehicle_id}: Virtual GPS data: location={location}, rotation={rotation}")
        
        if location is not None and rotation is not None:
            # Calculate velocity
            if self.prev_pos is not None and self.prev_time is not None:
                dt = current_time - self.prev_time
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

    def _update_physical_qcar_data(self, current_time):
        """Update using physical QCar API with EKF (similar to vehicle_control.py)."""
        if self.physical_qcar is None:
            return
            
        # Read from physical QCar sensors
        self.physical_qcar.read()
        
        # Get the current steering angle for control input
        delta = getattr(self, '_last_steering_angle', 0.0)
        
        if self.enable_steering_control and self.gps is not None:
            # GPS and EKF update
            if hasattr(self.gps, 'readGPS') and self.gps.readGPS():
                # GPS data available
                y_gps = np.array([
                    self.gps.position[0],
                    self.gps.position[1], 
                    self.gps.orientation[2]
                ])
                
                # Calculate dt
                dt = current_time - self.prev_time if self.prev_time is not None else 1.0/self.update_rate
                
                # Update EKF with GPS
                self.ekf.update(
                    [self.physical_qcar.motorTach, delta],
                    dt,
                    y_gps,
                    self.physical_qcar.gyroscope[2] if hasattr(self.physical_qcar, 'gyroscope') else 0.0
                )
                
                # Get EKF state estimate
                x = self.ekf.x_hat[0, 0]
                y = self.ekf.x_hat[1, 0]
                th = self.ekf.x_hat[2, 0]
                
                # Adjust position for vehicle front (like in vehicle_control.py)
                p = np.array([x, y]) + np.array([np.cos(th), np.sin(th)]) * 0.2
                
                self.current_pos = [p[0], p[1], 0.0]
                self.current_rot = [0.0, 0.0, th]
                self.velocity = self.physical_qcar.motorTach
                
                # Debug: Print physical QCar data for leader
                if self.is_leader and self.vehicle_id == 0:
                    print(f"Vehicle {self.vehicle_id}: Physical GPS data: pos=[{p[0]:.3f}, {p[1]:.3f}], "
                          f"th={th:.3f}, vel={self.velocity:.3f}")
                
            else:
                # No GPS, EKF prediction only
                dt = current_time - self.prev_time if self.prev_time is not None else 1.0/self.update_rate
                
                self.ekf.update(
                    [self.physical_qcar.motorTach, delta],
                    dt,
                    None,  # No GPS measurement
                    self.physical_qcar.gyroscope[2] if hasattr(self.physical_qcar, 'gyroscope') else 0.0
                )
                
                # Get EKF state estimate
                x = self.ekf.x_hat[0, 0]
                y = self.ekf.x_hat[1, 0]
                th = self.ekf.x_hat[2, 0]
                
                # Adjust position for vehicle front
                p = np.array([x, y]) + np.array([np.cos(th), np.sin(th)]) * 0.2
                
                self.current_pos = [p[0], p[1], 0.0]
                self.current_rot = [0.0, 0.0, th]
                self.velocity = self.physical_qcar.motorTach
        
        else:
            # No EKF, use direct motor tachometer
            self.velocity = self.physical_qcar.motorTach if hasattr(self.physical_qcar, 'motorTach') else 0.0
        
        # Update time tracking
        self.prev_time = current_time
        
        # Update GPS cache
        self.gps_data_cache.update({
            'position': self.current_pos.copy(),
            'rotation': self.current_rot.copy(),
            'velocity': self.velocity,
            'available': True,
            'timestamp': current_time,
            'last_update': current_time
        })

    def get_cached_gps_data(self):
        """Get cached GPS data."""
        return self.gps_data_cache.copy()
    
    def get_observer_state_direct(self) -> Optional[dict]:
        """
        Get state directly from observer without caching (factory method for observer state access).
        
        Returns:
            Observer state dictionary or None if observer not available
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
                    'velocity': local_state[3],
                    'gps_available': self.observer.gps_available if hasattr(self.observer, 'gps_available') else False,
                    'ekf_initialized': self.observer.ekf_initialized if hasattr(self.observer, 'ekf_initialized') else False,
                    'source': 'observer_local_direct'
                }
            else:
                # Fallback to the formatted method if local state is not available
                return self.observer.get_estimated_state_for_control()
        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: Failed to get observer state: {e}")
            return None
    
    def update_observer_state_cache(self, observer_state: dict):
        """Update the observer state cache with new data."""
        if observer_state:
            self.observer_state_cache.update({
                'position': observer_state.get('position', [0.0, 0.0, 0.0]),
                'rotation': observer_state.get('rotation', [0.0, 0.0, 0.0]),
                'velocity': observer_state.get('velocity', 0.0),
                'gps_available': observer_state.get('gps_available', False),
                'ekf_initialized': observer_state.get('ekf_initialized', False),
                'source': observer_state.get('source', 'unknown'),
                'timestamp': time.time()
            })
    
    def get_best_available_state(self) -> dict:
        """
        Factory method that returns the best available state estimate.
        Priority: Observer EKF > Observer fallback > Raw GPS
        
        Returns:
            Best available state estimate
        """
        # Try observer first
        observer_state = self.get_observer_state_direct()
        if observer_state and observer_state.get('ekf_initialized', False):
            return {
                'vehicle_id': self.vehicle_id,
                'position': observer_state['position'],
                'rotation': observer_state['rotation'],
                'velocity': observer_state['velocity'],
                'timestamp': time.time(),
                'is_leader': self.is_leader,
                'source': 'observer_ekf_best',
                'gps_available': observer_state.get('gps_available', False)
            }
        
        # Try cache next
        cache_age = time.time() - self.observer_state_cache.get('timestamp', 0.0)
        if cache_age < 1.0 and self.observer_state_cache.get('ekf_initialized', False):
            return {
                'vehicle_id': self.vehicle_id,
                'position': self.observer_state_cache['position'].copy(),
                'rotation': self.observer_state_cache['rotation'].copy(),
                'velocity': self.observer_state_cache['velocity'],
                'timestamp': time.time(),
                'is_leader': self.is_leader,
                'source': f'observer_cache_age_{cache_age:.3f}s',
                'gps_available': self.observer_state_cache.get('gps_available', False)
            }
        
        # Fallback to raw GPS
        return {
            'vehicle_id': self.vehicle_id,
            'position': self.current_pos.copy(),
            'rotation': self.current_rot.copy(),
            'velocity': self.velocity,
            'timestamp': time.time(),
            'is_leader': self.is_leader,
            'source': 'raw_gps_final_fallback',
            'gps_available': self.gps_data_cache.get('available', False)
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
        if self.use_physical_qcar and self.physical_qcar is not None:
            # Physical QCar mode - use write method like in vehicle_control.py
            self.physical_qcar.write(forward_speed, steering_angle)
            
            # Debug: Print physical control commands
            if self.is_leader and self.vehicle_id == 0:
                print(f"Vehicle {self.vehicle_id}: Physical control: u={forward_speed:.3f}, delta={steering_angle:.3f}")
        else:
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
            
            # Debug: Print virtual control commands  
            if self.is_leader and self.vehicle_id == 0:
                print(f"Vehicle {self.vehicle_id}: Virtual control: forward={forward_speed:.3f}, turn={steering_angle:.3f}")
        
        # Send state to followers (broadcast)
        self.comm_logger.info(f"Vehicle {self.vehicle_id}: Broadcasting state - pos={control_state['position']}, vel={control_state['velocity']:.3f}")
        self.comm.send_state_broadcast(control_state)
        
        # Log control timing for debugging
        if self.logger.isEnabledFor(logging.DEBUG):
            target_dt = 1.0 / self.update_rate
            timing_error = abs(dt - target_dt) / target_dt * 100
            self.logger.debug(f"Leader control using {control_state['source']} - "
                            f"GPS available: {control_state['gps_available']}, "
                            f"dt={dt:.4f}s (target={target_dt:.4f}s, error={timing_error:.1f}%)")
        # except Exception as e:
        #     self.logger.error(f"Leader control error: {e}")
        #     # Reset control input on error
        #     self.current_control_input = np.array([0.0, 0.0])

    def follower_control_logic(self):
        """Follower control logic that delegates to VehicleFollowerController."""
        if self.is_leader or self.follower_controller is None:
            print(f"Vehicle {self.vehicle_id}: Skipping follower control - is_leader={self.is_leader}, controller_exists={self.follower_controller is not None}")
            return
        
        # print(f"Vehicle {self.vehicle_id}: Executing follower control logic")
        try:
            # Use GPS-synchronized time for better prediction
            current_gps_time = self.gps_sync.get_synced_time()
            self.comm_logger.debug(f"Vehicle {self.vehicle_id}: Follower control - GPS time: {current_gps_time}")
            
            # Get the best available leader state data from validated queue
            leader_data = None
            
            # Try to get interpolated state for current time (most accurate)
            interpolated_state = self.get_interpolated_leader_state(current_gps_time)
            self.comm_logger.debug(f"Vehicle {self.vehicle_id}: Interpolated state: {interpolated_state}")
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
                self.comm_logger.info(f"Vehicle {self.vehicle_id}: Using fallback leader state: {self.leader_state}")
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
                    self.comm_logger.warning(f"Leader state too old ({state_age:.3f}s), stopping vehicle")
                    # Stop vehicle if leader data is too old
                    self.qcar.set_velocity_and_request_state(forward=0.0, turn=0.0, headlights=False,
                                                        leftTurnSignal=False,
                                                        rightTurnSignal=False,
                                                        brakeSignal=False,
                                                        reverseSignal=False)
                    self.current_control_input = np.array([0.0, 0.0])
                    return
            
            # No leader data available at all
            if leader_data is None:
                self.comm_logger.error(f"Vehicle {self.vehicle_id}: No leader data available, stopping vehicle")
                # self.logger.warning("No leader data available, stopping vehicle")
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
            # controller expects specific format for leader_data
            forward_speed, steering_angle = self.follower_controller.compute_control(
                current_pos=current_state['position'],
                current_rot=current_state['rotation'],
                current_velocity=current_state['velocity'],
                leader_pos=leader_data['position'],
                leader_rot=leader_data['rotation'],
                leader_velocity=leader_data['velocity'],
                leader_timestamp=leader_data['timestamp'],
                dt=1.0 / self.update_rate
            )
            # print(f"Vehicle {self.vehicle_id}: Applied control - forward: {forward_speed}, steering: {steering_angle}")

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

            # if self.logger.isEnabledFor(logging.DEBUG):
            #     self.logger.debug(f"Follower control: forward={forward_speed:.3f}, steering={steering_angle:.3f}, "
            #                     f"leader_pos={leader_data['position']}, current_pos={current_state['position']}")
                
        except Exception as e:
            self.comm_logger.error(f"Vehicle {self.vehicle_id}: Follower control error: {e}")
            # Reset control input on error
            self.current_control_input = np.array([0.0, 0.0])
            self.qcar.set_velocity_and_request_state(forward=0.0, turn=0.0)


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

    def process_received_state(self, received_state: dict):
        """Process a received state from leader."""
        self.comm_logger.info(f"Vehicle {self.vehicle_id}: Processing received state: {received_state}")
        success = self.state_queue.add_state(received_state, self.gps_sync)
        self.comm_logger.info(f"Vehicle {self.vehicle_id}: State added to queue: {success}")
        if not success:
            self.comm_logger.warning(f"Vehicle {self.vehicle_id}: State rejected by queue!")
        else:
            queue_stats = self.state_queue.get_queue_stats()
            self.comm_logger.debug(f"Vehicle {self.vehicle_id}: Queue size: {queue_stats['current_queue_size']}")
            
        # Also update leader_state for fallback
        if success and 'pos' in received_state:
            self.leader_state = received_state
            self.comm_logger.debug(f"Vehicle {self.vehicle_id}: Updated leader_state fallback")

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
                self.qcar.set_velocity_and_request_state(forward=0.0, turn=0.0)
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
            
            # Get sensor data (try to get from QCar like vehicle_control2.py)
            try:
                # Try to get motor tachometer from QCar (more accurate than velocity estimate)
                motor_tach = self.velocity  # Fallback to velocity estimate
                gyroscope_z = 0.0  # Could get from QCar if available
                
                # If QCar has sensor data available, use it
                # Note: This would require accessing QCar sensor data which might not be readily available in this context
                
            except Exception as sensor_error:
                # Fallback to estimated values
                motor_tach = self.velocity
                gyroscope_z = 0.0
            
            # Update observer local state
            estimated_state = self.observer.update_local_state(
                measured_state=measured_state,
                control_input=self.current_control_input,
                timestamp=current_time,
                motor_tach=motor_tach,
                gyroscope_z=gyroscope_z
            )
            
            # Use the returned estimated_state directly instead of making redundant call
            if estimated_state is not None and len(estimated_state) >= 4:
                # Convert numpy array [x, y, theta, v] to our cache format
                self.observer_state_cache.update({
                    'position': [estimated_state[0], estimated_state[1], 0.0],  # [x, y, z] format
                    'rotation': [0.0, 0.0, estimated_state[2]],  # [roll, pitch, yaw] format
                    'velocity': estimated_state[3],
                    'gps_available': gps_data['available'],
                    'ekf_initialized': self.observer.ekf_initialized if hasattr(self.observer, 'ekf_initialized') else True,
                    'source': 'observer_local_state',
                    'timestamp': current_time
                })
                
                # Debug log to verify cache is updated
                self.observer_logger.debug(f"Observer cache updated from local state: pos=({estimated_state[0]:.3f}, {estimated_state[1]:.3f}), "
                                         f"vel={estimated_state[3]:.3f}, "
                                         f"ekf_init={self.observer_state_cache['ekf_initialized']}")
            else:
                self.observer_logger.warning(f"Vehicle {self.vehicle_id}: Invalid estimated state from observer: {estimated_state}")
                
        except Exception as e:
            self.observer_logger.error(f"Vehicle {self.vehicle_id}: Observer update error: {e}")

    def handle_communication(self):
        """Handle non-blocking communication."""
        try:
            # Check for incoming messages
            received_data = self.comm.receive_state_non_blocking()
            if received_data:
                self.comm_logger.info(f"Vehicle {self.vehicle_id}: Received data from leader: {received_data}")
                self.process_received_state(received_data)
            else:
                # Only log this occasionally to avoid spam
                if not hasattr(self, '_last_no_data_log') or time.time() - self._last_no_data_log > 5.0:
                    self.comm_logger.debug(f"Vehicle {self.vehicle_id}: No data received from leader")
                    self._last_no_data_log = time.time()
                
        except Exception as e:
            self.comm_logger.error(f"Vehicle {self.vehicle_id}: Communication error: {e}")
            print(f"Vehicle {self.vehicle_id}: Communication error: {e}")  # Keep print for critical errors

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
                if self.use_physical_qcar and self.physical_qcar is not None:
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
                        
        except Exception as e:
            self.logger.error(f"Vehicle {self.vehicle_id}: Cleanup error: {e}")
            print(f"Vehicle {self.vehicle_id}: Cleanup error: {e}")
