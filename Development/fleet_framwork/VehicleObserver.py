import numpy as np
import threading
import time
import logging
from typing import Dict, List, Optional, Tuple
from collections import defaultdict
from hal.content.qcar_functions import QCarEKF
from md_logging_config import get_observer_logger


class VehicleObserver:
    """
    Observer system for vehicles in a fleet, providing both local state estimation
    and distributed state estimation for all vehicles in the fleet.
    
    Inspired by the observer.py implementation with Kalman filtering and distributed
    observer capabilities.
    """
    
    def __init__(self, vehicle_id: int, fleet_size: int, config=None, logger=None):
        """
        Initialize the Vehicle Observer.
        
        Args:
            vehicle_id: ID of the host vehicle
            fleet_size: Total number of vehicles in the fleet
            config: Configuration object containing observer parameters
            logger: Logger instance for this observer
        """
        self.vehicle_id = vehicle_id
        self.fleet_size = fleet_size
        self.config = config
        # Use specialized observer logger with timing precision
        self.logger = logger if logger is not None else get_observer_logger(vehicle_id)
        
        # State dimensions: [x, y, theta, v] - position, orientation, velocity
        self.state_dim = 4
        self.control_dim = 2  # [steering, acceleration]
        
        # Observer configuration
        self.observer_config = self._get_observer_config()
        
        # Local state estimation
        self.local_state = np.zeros(self.state_dim)
        self.local_state_prev = np.zeros(self.state_dim)
        
        # Distributed state estimation - estimates for all vehicles
        self.fleet_states = np.zeros((self.state_dim, self.fleet_size))
        self.fleet_states_prev = np.zeros((self.state_dim, self.fleet_size))
        
        # Initialize the host vehicle's state in the fleet estimates
        self.fleet_states[:, self.vehicle_id] = self.local_state
        
        # QCarEKF for local state estimation (inspired by VehicleLeaderController)
        self.ekf = None
        self.ekf_initialized = False
        
        # Kalman filter matrices for local estimation (fallback if QCarEKF not used)
        if self.observer_config["local_observer_type"] == "kalman":
            self.P_local = np.eye(self.state_dim) * 0.1  # Local covariance matrix
            self.R_local = np.diag([0.01, 0.01, 0.0003, 0.01])  # Measurement noise
            self.Q_local = np.diag([0.005, 0.005, 0.001, 0.01])  # Process noise
        
        # GPS availability tracking
        self.gps_available = False
        self.last_gps_update = 0.0
        
        # Distributed observer parameters
        self.distributed_weights = self._initialize_distributed_weights()
        self.communication_graph = self._initialize_communication_graph()
        
        # Data storage for received states from other vehicles
        self.received_states = defaultdict(list)  # vehicle_id -> list of (timestamp, state)
        self.received_controls = defaultdict(list)  # vehicle_id -> list of (timestamp, control)
        
        # Timing parameters
        self.dt = self.observer_config.get("dt", 0.01)  # 100 Hz default
        self.max_state_age = 1.0  # Maximum age of received states to use (seconds)
        
        # Tolerances for state validation
        self.tolerances = np.array([5.0, 2.0, np.deg2rad(8), 2.0])  # [x, y, theta, v]
        
        # Logging and monitoring
        self.estimation_log = []
        self.validation_log = []
        
        # Thread safety
        self.lock = threading.RLock()
        
        self.logger.info(f"VehicleObserver initialized for vehicle {vehicle_id} "
                        f"(Fleet size: {fleet_size}, Observer type: {self.observer_config['local_observer_type']})")
    
    def _get_observer_config(self) -> dict:
        """Get observer configuration with defaults."""
        default_config = {
            "local_observer_type": "kalman",  # "kalman", "luenberger", or "direct"
            "enable_distributed": True,
            "distributed_observer_type": "consensus",
            "dt": 0.01,
            "enable_noise_measurement": False,
            "enable_prediction": True,
            "consensus_gain": 0.1
        }
        
        if self.config and hasattr(self.config, 'observer'):
            for key, value in self.config.observer.items():
                default_config[key] = value
        
        return default_config
    
    def _initialize_distributed_weights(self) -> np.ndarray:
        """Initialize weights for distributed observer consensus."""
        # Simple uniform weighting for now
        weights = np.ones(self.fleet_size + 1) / (self.fleet_size + 1)
        weights[0] = 0.5  # Weight for local measurement
        weights[1:] = 0.5 / self.fleet_size  # Weights for other vehicles
        
        return weights
    
    def _initialize_communication_graph(self) -> np.ndarray:
        """Initialize communication graph adjacency matrix."""
        # For now, assume all vehicles can communicate with each other
        graph = np.ones((self.fleet_size, self.fleet_size))
        np.fill_diagonal(graph, 0)  # No self-loops
        
        return graph
    
    def initialize_ekf(self, initial_pose: Optional[np.ndarray] = None):
        """
        Initialize the QCarEKF for local state estimation.
        
        Args:
            initial_pose: Initial pose [x, y, theta] or None for default [0, 0, 0]
        """
        ekf_init_start = time.perf_counter()
        
        if initial_pose is None:
            initial_pose = np.array([0.0, 0.0, 0.0])  # [x, y, theta]
        
        try:
            # Initialize QCarEKF with initial pose (similar to VehicleLeaderController)
            self.ekf = QCarEKF(x_0=initial_pose)
            self.ekf_initialized = True
            self.local_state[:3] = initial_pose  # Set x, y, theta
            self.local_state[3] = 0.0  # Initialize velocity to 0
            
            ekf_init_time = (time.perf_counter() - ekf_init_start) * 1000
            
            self.logger.info(f"EKF_INIT: Success in {ekf_init_time:.3f}ms, "
                           f"InitialPose=({initial_pose[0]:.3f},{initial_pose[1]:.3f}), "
                           f"InitialTheta={np.rad2deg(initial_pose[2]):.1f}°")
            
            self.logger.info(f"QCarEKF initialized for vehicle {self.vehicle_id} "
                           f"with initial pose: {initial_pose}")
            
        except Exception as e:
            ekf_init_time = (time.perf_counter() - ekf_init_start) * 1000
            self.logger.error(f"EKF_INIT: Failed after {ekf_init_time:.3f}ms - {e}")
            self.logger.error(f"Failed to initialize QCarEKF: {e}")
            self.ekf_initialized = False
            self.ekf = None
    
    def get_system_matrices(self, state: np.ndarray, control: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get linearized system matrices A and B for the current state.
        
        Args:
            state: Current state [x, y, theta, v]
            control: Current control input [steering, acceleration]
            
        Returns:
            A: State transition matrix
            B: Control input matrix
        """
        x, y, theta, v = state
        dt = self.dt
        
        # Linearized bicycle model matrices
        A = np.array([
            [1, 0, -v * np.sin(theta) * dt, np.cos(theta) * dt],
            [0, 1,  v * np.cos(theta) * dt, np.sin(theta) * dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        B = np.array([
            [0, 0],
            [0, 0],
            [0, dt],  # steering affects heading rate
            [dt, 0]   # acceleration affects velocity
        ])
        
        return A, B
    
    def update_local_state(self, measured_state: Optional[np.ndarray], control_input: np.ndarray, 
                          timestamp: float, motor_tach: float = 0.0, gyroscope_z: float = 0.0) -> np.ndarray:
        """
        Update local state estimation using QCarEKF or fallback methods.
        
        Args:
            measured_state: GPS measurement [x, y, theta, v] or None if GPS not available
            control_input: Applied control input [steering, acceleration]
            timestamp: GPS-synchronized timestamp
            motor_tach: Motor tachometer reading (velocity measurement)
            gyroscope_z: Z-axis gyroscope reading (angular velocity)
            
        Returns:
            Updated local state estimate
        """
        # Start timing for observer performance analysis
        observer_start_time = time.perf_counter()
        
        with self.lock:
            # Step 1: Time calculation and validation
            dt_calc_start = time.perf_counter()
            if hasattr(self, 'last_update_time'):
                dt = timestamp - self.last_update_time
                dt = max(0.001, min(0.1, dt))  # Clamp dt between 1ms and 100ms
            else:
                dt = self.dt
            self.last_update_time = timestamp
            dt_calc_time = (time.perf_counter() - dt_calc_start) * 1000
            
            # Step 2: Observer method selection and state update
            state_update_start = time.perf_counter()
            
            # Use QCarEKF if initialized and enabled
            if (self.observer_config["local_observer_type"] == "kalman" and 
                self.ekf_initialized and self.ekf is not None):
                
                self.local_state = self._qcar_ekf_update(
                    measured_state, control_input, dt, motor_tach, gyroscope_z
                )
                method_used = "qcar_ekf"
                
            elif self.observer_config["local_observer_type"] == "kalman":
                # Fallback to custom Kalman filter if QCarEKF not available
                if measured_state is not None:
                    self.local_state = self._kalman_filter_update(measured_state, control_input, dt)
                    method_used = "kalman_update"
                else:
                    # Prediction only
                    self.local_state = self._kalman_prediction_only(control_input, dt)
                    method_used = "kalman_prediction"
                    
            elif self.observer_config["local_observer_type"] == "luenberger":
                if measured_state is not None:
                    self.local_state = self._luenberger_observer_update(measured_state, control_input, dt)
                    method_used = "luenberger_update"
                else:
                    # Prediction only
                    A, B = self.get_system_matrices(self.local_state, control_input)
                    self.local_state = A @ self.local_state + B @ control_input
                    method_used = "luenberger_prediction"
                    
            else:  # direct measurement
                if measured_state is not None:
                    self.local_state = measured_state.copy()
                    # Update velocity from motor tach if available
                    if motor_tach != 0.0:
                        self.local_state[3] = motor_tach
                    method_used = "direct_measurement"
                else:
                    method_used = "direct_hold"
                # If no measurement available, keep previous state

            # if self.vehicle_id == 0 :
            #     print(f"Vehicle {self.vehicle_id}: State updated using {method_used}")

            state_update_time = (time.perf_counter() - state_update_start) * 1000
            
            # Step 3: GPS availability tracking
            gps_tracking_start = time.perf_counter()
            self.gps_available = measured_state is not None
            if self.gps_available:
                self.last_gps_update = timestamp
            gps_tracking_time = (time.perf_counter() - gps_tracking_start) * 1000
            
            # Step 4: Fleet state update
            fleet_update_start = time.perf_counter()
            self.fleet_states[:, self.vehicle_id] = self.local_state
            fleet_update_time = (time.perf_counter() - fleet_update_start) * 1000
            
            # Step 5: Logging and bookkeeping
            logging_start = time.perf_counter()
            self.estimation_log.append({
                'timestamp': timestamp,
                'vehicle_id': self.vehicle_id,
                'local_state': self.local_state.copy(),
                'measured_state': measured_state.copy() if measured_state is not None else None,
                'control_input': control_input.copy(),
                'gps_available': self.gps_available,
                'dt': dt
            })
            logging_time = (time.perf_counter() - logging_start) * 1000
            
            # Calculate total observer time
            total_observer_time = (time.perf_counter() - observer_start_time) * 1000
            
            # Log detailed timing information for observer performance analysis
            self.logger.info(f"OBS_TIMING: Total={total_observer_time:.3f}ms, DTCalc={dt_calc_time:.3f}ms, "
                           f"StateUpdate={state_update_time:.3f}ms, GPSTrack={gps_tracking_time:.3f}ms, "
                           f"FleetUpdate={fleet_update_time:.3f}ms, Logging={logging_time:.3f}ms")
            
            # Log observer state and method information
            self.logger.info(f"OBS_STATE: Method={method_used}, GPS_avail={self.gps_available}, "
                           f"EKF_init={self.ekf_initialized}, DT={dt:.4f}s, "
                           f"Pos=({self.local_state[0]:.3f},{self.local_state[1]:.3f}), "
                           f"Theta={np.rad2deg(self.local_state[2]):.1f}°, Vel={self.local_state[3]:.3f}")
            
            # Log input data details
            if measured_state is not None:
                self.logger.info(f"OBS_INPUT: GPS=({measured_state[0]:.3f},{measured_state[1]:.3f}), "
                               f"GPS_Theta={np.rad2deg(measured_state[2]):.1f}°, GPS_Vel={measured_state[3]:.3f}, "
                               f"MotorTach={motor_tach:.3f}, Gyro={gyroscope_z:.3f}")
            else:
                self.logger.info(f"OBS_INPUT: GPS=None, MotorTach={motor_tach:.3f}, Gyro={gyroscope_z:.3f}")
            
            # Log control input
            self.logger.info(f"OBS_CONTROL: Steering={control_input[0]:.3f}, Accel={control_input[1]:.3f}")
            
            # Performance warning if observer update is taking too long
            if total_observer_time > 5.0:  # More than 5ms
                self.logger.warning(f"OBS_PERFORMANCE: Observer update slow ({total_observer_time:.3f}ms) - "
                                  f"Target: <5ms for 100Hz control loop")
            
            self.logger.debug(f"Local state updated: pos=({self.local_state[0]:.3f}, {self.local_state[1]:.3f}), "
                            f"theta={np.rad2deg(self.local_state[2]):.1f}°, v={self.local_state[3]:.3f}, "
                            f"GPS={self.gps_available}")
            
            return self.local_state.copy()
    
    def _qcar_ekf_update(self, measured_state: Optional[np.ndarray], control_input: np.ndarray, 
                        dt: float, motor_tach: float, gyroscope_z: float) -> np.ndarray:
        """
        Update state using QCarEKF (inspired by VehicleLeaderController implementation).
        
        Args:
            measured_state: GPS measurement [x, y, theta, v] or None if not available
            control_input: Control input [steering, acceleration]
            dt: Time step
            motor_tach: Motor tachometer reading
            gyroscope_z: Z-axis gyroscope reading
            
        Returns:
            Updated state estimate [x, y, theta, v]
        """
        # Start timing for EKF operation
        ekf_start_time = time.perf_counter()
        
        try:
            # Step 1: Prepare control input
            control_prep_start = time.perf_counter()
            steering = control_input[0]
            control_prep_time = (time.perf_counter() - control_prep_start) * 1000
            
            # Step 2: EKF Update
            ekf_update_start = time.perf_counter()
            if measured_state is not None:
                # QCarEKF expects GPS measurement as [x, y, theta]
                y_gps = np.array([measured_state[0], measured_state[1], measured_state[2]])
                
                # Update EKF with GPS measurement
                self.ekf.update([motor_tach, steering], dt, y_gps, gyroscope_z)
                update_type = "with_gps"
            else:
                # Update EKF without GPS measurement (prediction only)
                self.ekf.update([motor_tach, steering], dt, None, gyroscope_z)
                update_type = "prediction_only"
            ekf_update_time = (time.perf_counter() - ekf_update_start) * 1000
            
            # Step 3: Extract state from EKF
            state_extract_start = time.perf_counter()
            x = self.ekf.x_hat[0, 0]
            y = self.ekf.x_hat[1, 0] 
            theta = self.ekf.x_hat[2, 0]
            
            # Use motor tachometer for velocity (more reliable than GPS-derived velocity)
            v = motor_tach
            
            # Return state in our format [x, y, theta, v]
            estimated_state = np.array([x, y, theta, v])
            state_extract_time = (time.perf_counter() - state_extract_start) * 1000
            
            # Calculate total EKF time
            total_ekf_time = (time.perf_counter() - ekf_start_time) * 1000
            
            # Log detailed EKF timing
            self.logger.info(f"EKF_TIMING: Total={total_ekf_time:.3f}ms, ControlPrep={control_prep_time:.3f}ms, "
                           f"EKFUpdate={ekf_update_time:.3f}ms, StateExtract={state_extract_time:.3f}ms")
            
            # Log EKF operation details
            self.logger.info(f"EKF_OPERATION: Type={update_type}, DT={dt:.4f}s, "
                           f"Input=[MotorTach={motor_tach:.3f}, Steering={steering:.3f}, Gyro={gyroscope_z:.3f}]")
            
            # Log EKF state output
            self.logger.info(f"EKF_STATE: Out=({x:.3f},{y:.3f}), Theta={np.rad2deg(theta):.1f}°, Vel={v:.3f}")
            
            # if measured_state is not None:
            #     # Log GPS vs EKF comparison
            #     gps_x, gps_y, gps_theta = measured_state[0], measured_state[1], measured_state[2]
            #     pos_error = np.sqrt((x - gps_x)**2 + (y - gps_y)**2)
            #     theta_error = abs(theta - gps_theta)
            #     self.logger.info(f"EKF_GPS_COMPARE: PosError={pos_error:.3f}m, ThetaError={np.rad2deg(theta_error):.1f}°")
            
            # Performance warning
            if total_ekf_time > 3.0:  # More than 3ms for EKF update
                self.logger.warning(f"EKF_PERFORMANCE: EKF update slow ({total_ekf_time:.3f}ms)")
            
            self.logger.debug(f"QCarEKF update: GPS={measured_state is not None}, "
                            f"motor_tach={motor_tach:.3f}, steering={steering:.3f}")
            
            return estimated_state
            
        except Exception as e:
            total_time = (time.perf_counter() - ekf_start_time) * 1000
            self.logger.error(f"EKF_ERROR: QCarEKF update failed after {total_time:.3f}ms - {e}")
            # Fallback to previous state
            return self.local_state.copy()
    
    def _kalman_filter_update(self, measurement: np.ndarray, control: np.ndarray, dt: float) -> np.ndarray:
        """Kalman filter update for local state estimation (fallback method)."""
        # Update dt for system matrices
        self.dt = dt
        A, B = self.get_system_matrices(self.local_state, control)
        
        # Prediction step
        x_pred = A @ self.local_state + B @ control
        P_pred = A @ self.P_local @ A.T + self.Q_local
        
        # Update step
        C = np.eye(self.state_dim)  # Direct state measurement
        y = measurement - C @ x_pred  # Innovation
        S = C @ P_pred @ C.T + self.R_local  # Innovation covariance
        K = P_pred @ C.T @ np.linalg.inv(S)  # Kalman gain
        
        # State and covariance update
        x_updated = x_pred + K @ y
        self.P_local = (np.eye(self.state_dim) - K @ C) @ P_pred
        
        return x_updated
    
    def _kalman_prediction_only(self, control: np.ndarray, dt: float) -> np.ndarray:
        """Kalman filter prediction step only (when no GPS measurement available)."""
        # Update dt for system matrices
        self.dt = dt
        A, B = self.get_system_matrices(self.local_state, control)
        
        # Prediction step only
        x_pred = A @ self.local_state + B @ control
        self.P_local = A @ self.P_local @ A.T + self.Q_local
        
        return x_pred
    
    def _luenberger_observer_update(self, measurement: np.ndarray, control: np.ndarray, dt: float) -> np.ndarray:
        """Luenberger observer update for local state estimation."""
        # Update dt for system matrices
        self.dt = dt
        A, B = self.get_system_matrices(self.local_state, control)
        
        # Observer gain (simplified - could be optimized)
        L = np.eye(self.state_dim) * 0.1
        
        # Observer update
        x_updated = A @ self.local_state + B @ control + L @ (measurement - self.local_state)
        
        return x_updated
    
    def add_received_state(self, sender_id: int, state: np.ndarray, control: np.ndarray, 
                          timestamp: float) -> bool:
        """
        Add a received state from another vehicle for distributed estimation.
        
        Args:
            sender_id: ID of the vehicle that sent the state
            state: Received state [x, y, theta, v]
            control: Received control input [steering, acceleration]
            timestamp: GPS-synchronized timestamp
            
        Returns:
            True if state was accepted, False if rejected (too old, invalid, etc.)
        """
        current_time = time.time()
        
        # Validate timestamp
        if current_time - timestamp > self.max_state_age:
            self.logger.debug(f"Rejected old state from vehicle {sender_id} "
                            f"(age: {current_time - timestamp:.3f}s)")
            return False
        
        # Validate state dimensions
        if len(state) != self.state_dim or len(control) != self.control_dim:
            self.logger.warning(f"Invalid state/control dimensions from vehicle {sender_id}")
            return False
        
        with self.lock:
            # Clean old data
            self._cleanup_old_data(sender_id, current_time)
            
            # Add new data
            self.received_states[sender_id].append((timestamp, state.copy()))
            self.received_controls[sender_id].append((timestamp, control.copy()))
            
            # Keep only recent data
            max_history = 100
            if len(self.received_states[sender_id]) > max_history:
                self.received_states[sender_id] = self.received_states[sender_id][-max_history:]
                self.received_controls[sender_id] = self.received_controls[sender_id][-max_history:]
        
        self.logger.debug(f"Added state from vehicle {sender_id}: "
                         f"pos=({state[0]:.3f}, {state[1]:.3f}), v={state[3]:.3f}")
        return True
    
    def _cleanup_old_data(self, sender_id: int, current_time: float):
        """Remove old data that exceeds the maximum age."""
        cutoff_time = current_time - self.max_state_age
        
        # Filter states
        self.received_states[sender_id] = [
            (ts, state) for ts, state in self.received_states[sender_id]
            if ts >= cutoff_time
        ]
        
        # Filter controls
        self.received_controls[sender_id] = [
            (ts, control) for ts, control in self.received_controls[sender_id]
            if ts >= cutoff_time
        ]
    
    def update_distributed_estimates(self, timestamp: float) -> np.ndarray:
        """
        Update distributed state estimates for all vehicles in the fleet.
        
        Args:
            timestamp: Current GPS-synchronized timestamp
            
        Returns:
            Updated fleet state estimates
        """
        if not self.observer_config["enable_distributed"]:
            return self.fleet_states.copy()
        
        # Start timing for distributed observer performance
        distributed_start_time = time.perf_counter()
        
        with self.lock:
            fleet_states_new = self.fleet_states.copy()
            
            # Step 1: Update estimates for each vehicle
            vehicle_update_times = []
            
            for vehicle_id in range(self.fleet_size):
                vehicle_start = time.perf_counter()
                
                if vehicle_id == self.vehicle_id:
                    # Use local estimate for own vehicle
                    fleet_states_new[:, vehicle_id] = self.local_state
                    update_method = "local"
                else:
                    # Distributed observer for other vehicles
                    estimated_state = self._distributed_observer_update(
                        vehicle_id, timestamp, fleet_states_new
                    )
                    fleet_states_new[:, vehicle_id] = estimated_state
                    update_method = "distributed"
                
                vehicle_time = (time.perf_counter() - vehicle_start) * 1000
                vehicle_update_times.append(vehicle_time)
                
                # Log individual vehicle update
                state = fleet_states_new[:, vehicle_id]
                self.logger.debug(f"DIST_VEHICLE: ID={vehicle_id}, Method={update_method}, "
                                f"Time={vehicle_time:.3f}ms, "
                                f"Pos=({state[0]:.3f},{state[1]:.3f}), Vel={state[3]:.3f}")
            
            # Step 2: Update fleet states
            state_copy_start = time.perf_counter()
            self.fleet_states_prev = self.fleet_states.copy()
            self.fleet_states = fleet_states_new
            state_copy_time = (time.perf_counter() - state_copy_start) * 1000
            
            # Calculate total distributed observer time
            total_distributed_time = (time.perf_counter() - distributed_start_time) * 1000
            
            # Log distributed observer timing
            max_vehicle_time = max(vehicle_update_times) if vehicle_update_times else 0
            avg_vehicle_time = sum(vehicle_update_times) / len(vehicle_update_times) if vehicle_update_times else 0
            
            self.logger.info(f"DIST_TIMING: Total={total_distributed_time:.3f}ms, "
                           f"MaxVehicle={max_vehicle_time:.3f}ms, AvgVehicle={avg_vehicle_time:.3f}ms, "
                           f"StateCopy={state_copy_time:.3f}ms, Vehicles={self.fleet_size}")
            
            # Log fleet state summary
            local_pos = self.fleet_states[:2, self.vehicle_id]
            self.logger.info(f"DIST_FLEET: LocalPos=({local_pos[0]:.3f},{local_pos[1]:.3f}), "
                           f"FleetSize={self.fleet_size}, Timestamp={timestamp:.3f}")
            
            # Performance warning
            if total_distributed_time > 10.0:  # More than 10ms
                self.logger.warning(f"DIST_PERFORMANCE: Distributed update slow ({total_distributed_time:.3f}ms) - "
                                  f"Target: <10ms for {self.fleet_size} vehicles")
            
            self.logger.debug(f"Distributed estimates updated for {self.fleet_size} vehicles")
            
            return self.fleet_states.copy()
    
    def _distributed_observer_update(self, target_vehicle_id: int, timestamp: float, 
                                   current_estimates: np.ndarray) -> np.ndarray:
        """
        Update estimate for a specific vehicle using distributed observer.
        
        Args:
            target_vehicle_id: ID of vehicle to estimate
            timestamp: Current timestamp
            current_estimates: Current state estimates for all vehicles
            
        Returns:
            Updated state estimate for the target vehicle
        """
        # Get the most recent state from the target vehicle
        received_state = self._get_latest_state(target_vehicle_id, timestamp)
        received_control = self._get_latest_control(target_vehicle_id, timestamp)
        
        if received_state is None:
            # No recent data available, use prediction based on previous estimate
            if received_control is not None:
                A, B = self.get_system_matrices(
                    current_estimates[:, target_vehicle_id], received_control
                )
                predicted_state = A @ current_estimates[:, target_vehicle_id] + B @ received_control
            else:
                # No control data either, just use previous estimate
                predicted_state = current_estimates[:, target_vehicle_id]
            
            return predicted_state
        
        # Consensus-based distributed observer
        consensus_term = np.zeros(self.state_dim)
        
        # Compute consensus with neighboring vehicles
        for neighbor_id in range(self.fleet_size):
            if (neighbor_id != self.vehicle_id and 
                neighbor_id != target_vehicle_id and
                self.communication_graph[self.vehicle_id, neighbor_id] > 0):
                
                # Get neighbor's estimate of the target vehicle
                neighbor_estimate = current_estimates[:, target_vehicle_id]
                my_estimate = current_estimates[:, target_vehicle_id]
                
                consensus_term += self.distributed_weights[neighbor_id + 1] * (
                    neighbor_estimate - my_estimate
                )
        
        # Combine local measurement, prediction, and consensus
        if received_control is not None:
            A, B = self.get_system_matrices(current_estimates[:, target_vehicle_id], received_control)
            predicted_state = A @ current_estimates[:, target_vehicle_id] + B @ received_control
        else:
            predicted_state = current_estimates[:, target_vehicle_id]
        
        # Apply consensus update
        gain = self.observer_config["consensus_gain"]
        measurement_weight = self.distributed_weights[0]
        
        updated_state = (predicted_state + 
                        consensus_term + 
                        measurement_weight * gain * (received_state - predicted_state))
        
        return updated_state
    
    def _get_latest_state(self, vehicle_id: int, timestamp: float) -> Optional[np.ndarray]:
        """Get the most recent state from a specific vehicle."""
        if vehicle_id not in self.received_states or not self.received_states[vehicle_id]:
            return None
        
        # Find the most recent state within the time window
        cutoff_time = timestamp - self.max_state_age
        valid_states = [
            (ts, state) for ts, state in self.received_states[vehicle_id]
            if ts >= cutoff_time
        ]
        
        if not valid_states:
            return None
        
        # Return the most recent state
        _, latest_state = max(valid_states, key=lambda x: x[0])
        return latest_state
    
    def _get_latest_control(self, vehicle_id: int, timestamp: float) -> Optional[np.ndarray]:
        """Get the most recent control input from a specific vehicle."""
        if vehicle_id not in self.received_controls or not self.received_controls[vehicle_id]:
            return None
        
        # Find the most recent control within the time window
        cutoff_time = timestamp - self.max_state_age
        valid_controls = [
            (ts, control) for ts, control in self.received_controls[vehicle_id]
            if ts >= cutoff_time
        ]
        
        if not valid_controls:
            return None
        
        # Return the most recent control
        _, latest_control = max(valid_controls, key=lambda x: x[0])
        return latest_control
    
    def get_local_state(self) -> np.ndarray:
        """Get the current local state estimate."""
        with self.lock:
            return self.local_state.copy()
    
    def get_fleet_states(self) -> np.ndarray:
        """Get the current fleet state estimates."""
        with self.lock:
            return self.fleet_states.copy()
    
    def get_vehicle_state(self, vehicle_id: int) -> Optional[np.ndarray]:
        """Get the state estimate for a specific vehicle."""
        if 0 <= vehicle_id < self.fleet_size:
            with self.lock:
                return self.fleet_states[:, vehicle_id].copy()
        return None
    
    def validate_state_estimate(self, vehicle_id: int, true_state: np.ndarray) -> dict:
        """
        Validate the state estimate against ground truth.
        
        Args:
            vehicle_id: ID of vehicle to validate
            true_state: Ground truth state
            
        Returns:
            Dictionary with validation results
        """
        estimated_state = self.get_vehicle_state(vehicle_id)
        if estimated_state is None:
            return {'valid': False, 'error': 'No estimate available'}
        
        # Calculate absolute errors
        errors = np.abs(estimated_state - true_state)
        
        # Check if errors are within tolerances
        is_valid = np.all(errors <= self.tolerances)
        violating_elements = np.where(errors > self.tolerances)[0].tolist()
        
        validation_result = {
            'valid': is_valid,
            'errors': errors,
            'tolerances': self.tolerances,
            'violating_elements': violating_elements,
            'max_error': np.max(errors),
            'rms_error': np.sqrt(np.mean(errors**2))
        }
        
        # Log validation results
        self.validation_log.append({
            'timestamp': time.time(),
            'vehicle_id': vehicle_id,
            'validation': validation_result
        })
        
        if not is_valid:
            self.logger.warning(f"State validation failed for vehicle {vehicle_id}: "
                              f"errors={errors}, violating={violating_elements}")
        
        return validation_result
    
    def get_observer_stats(self) -> dict:
        """Get statistics about the observer performance."""
        with self.lock:
            stats = {
                'vehicle_id': self.vehicle_id,
                'fleet_size': self.fleet_size,
                'config': self.observer_config,
                'local_state': self.local_state.copy(),
                'fleet_states': self.fleet_states.copy(),
                'received_data_counts': {
                    vid: len(states) for vid, states in self.received_states.items()
                },
                'estimation_log_size': len(self.estimation_log),
                'validation_log_size': len(self.validation_log)
            }
            
            # Add Kalman filter covariance if using Kalman filter
            if self.observer_config["local_observer_type"] == "kalman":
                stats['kalman_covariance'] = self.P_local.copy()
                stats['kalman_trace'] = np.trace(self.P_local)
            
            return stats
    
    def reset_observer(self, initial_state: Optional[np.ndarray] = None):
        """Reset the observer to initial conditions."""
        with self.lock:
            if initial_state is not None:
                self.local_state = initial_state.copy()
                self.fleet_states[:, self.vehicle_id] = initial_state.copy()
                # Reinitialize EKF with new initial pose if using QCarEKF
                if (self.observer_config["local_observer_type"] == "kalman" and 
                    len(initial_state) >= 3):
                    self.initialize_ekf(initial_state[:3])  # [x, y, theta]
            else:
                self.local_state = np.zeros(self.state_dim)
                self.fleet_states = np.zeros((self.state_dim, self.fleet_size))
                # Reinitialize EKF with default pose
                if self.observer_config["local_observer_type"] == "kalman":
                    self.initialize_ekf()
            
            # Reset Kalman filter covariance
            if self.observer_config["local_observer_type"] == "kalman":
                self.P_local = np.eye(self.state_dim) * 0.1
            
            # Reset GPS tracking
            self.gps_available = False
            self.last_gps_update = 0.0
            
            # Clear data history
            self.received_states.clear()
            self.received_controls.clear()
            self.estimation_log.clear()
            self.validation_log.clear()
            
            self.logger.info(f"Observer reset for vehicle {self.vehicle_id}")
    
    def get_estimated_state_for_control(self) -> dict:
        """
        Get estimated state in format suitable for vehicle controllers.
        
        Returns:
            Dictionary with estimated state information
        """
        with self.lock:
            # Calculate time since last GPS update
            time_since_gps = time.time() - self.last_gps_update if self.last_gps_update > 0 else float('inf')
            
            # Determine state source
            if self.gps_available and time_since_gps < 0.5:
                source = "gps_recent"
            elif self.ekf_initialized:
                source = "ekf_estimate"
            else:
                source = "fallback"
            
            result = {
                'position': [self.local_state[0], self.local_state[1], 0.0],  # [x, y, z] format
                'rotation': [0.0, 0.0, self.local_state[2]],  # [roll, pitch, yaw] format  
                'velocity': self.local_state[3],
                'estimated': True,
                'gps_available': self.gps_available,
                'time_since_gps': time_since_gps,
                'ekf_initialized': self.ekf_initialized,
                'source': source
            }
            
            # Log state request for control (at debug level to avoid spam)
            self.logger.debug(f"CONTROL_STATE_REQUEST: Source={source}, "
                            f"GPS_available={self.gps_available}, "
                            f"Time_since_GPS={time_since_gps:.3f}s, "
                            f"EKF_init={self.ekf_initialized}")
            
            return result
