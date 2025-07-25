import socket
import threading
import time
import json
import math
import logging
from typing import Dict, List, Any, Optional
from collections import deque
from qvl.qcar2 import QLabsQCar2
from src.Controller.CACC import CACC
from src.Controller.idm_control import IDMControl
from src.Controller.DummyController import DummyController, DummyVehicle
from pal.utilities.math import wrap_to_pi


class PlatoonVehicle:
    """
    Distributed platoon vehicle that handles its own control, communication, and data sharing.
    Each vehicle runs independently with separate threads for control, sending, and receiving.
    """
    
    def __init__(self, vehicle_id: int, qcar: QLabsQCar2, controller_type: str = "CACC", 
                 is_leader: bool = False, config=None, communication_config=None):
        """
        Initialize a platoon vehicle.
        
        Args:
            vehicle_id: Unique identifier for this vehicle
            qcar: QLabsQCar2 instance for this vehicle
            controller_type: Type of controller ("CACC" or "IDM")
            is_leader: Whether this vehicle is the leader
            config: Configuration object containing vehicle parameters
            communication_config: Dict with communication settings (ports, IPs, etc.)
        """
        self.vehicle_id = vehicle_id
        self.qcar = qcar
        self.controller_type = controller_type
        self.is_leader = is_leader
        self.config = config
        
        # Communication configuration
        comm_config = communication_config or {}
        self.base_port = comm_config.get('base_port', 5000)
        self.broadcast_ip = comm_config.get('broadcast_ip', '127.0.0.1')
        self.send_port = self.base_port + vehicle_id
        self.recv_port = self.base_port + 100 + vehicle_id
        
        # Threading and state
        self.running = False
        self.threads = []
        self.lock = threading.Lock()
        
        # Vehicle state
        self.current_pos = [0, 0, 0]
        self.current_rot = [0, 0, 0]
        self.velocity = 0.0
        self.prev_pos = None
        self.prev_time = None
        
        # Platoon state storage
        self.platoon_data = {}  # Store data from all vehicles
        self.last_update_times = {}  # Track when we last heard from each vehicle
        self.data_queue = deque(maxlen=10)  # Keep recent state history
        
        # Control parameters
        self.update_rate = 20  # Hz for control loop
        self.send_rate = 10    # Hz for data transmission
        self.recv_rate = 50    # Hz for data reception
        self.max_steering = config.max_steering if config else 0.6
        self.lookahead_distance = config.lookahead_distance if config else 0.4
        self.k_steering = 2.0
        
        # Initialize controller
        self._init_controller()
        
        # Setup communication
        self._init_communication()
        
        # Logging
        self.logger = logging.getLogger(f"PlatoonVehicle_{vehicle_id}")
        
        # Leader-specific initialization
        if self.is_leader:
            self._init_leader_control()
        
        self.logger.info(f"Platoon vehicle {vehicle_id} initialized (Leader: {is_leader})")
    
    def _init_controller(self):
        """Initialize the appropriate controller for this vehicle."""
        if self.config is not None:
            dummy_params = self.config.get_dummy_controller_params(self.vehicle_id)
            dummy_controller = DummyController(self.vehicle_id, dummy_params)
        else:
            dummy_controller = DummyController(self.vehicle_id)
            
        if self.controller_type == "CACC":
            self.controller = CACC(dummy_controller)
        elif self.controller_type == "IDM":
            self.controller = IDMControl(dummy_controller)
        else:
            raise ValueError(f"Unknown controller type: {self.controller_type}")
        
        # Create dummy vehicle for leader tracking (for followers)
        self.dummy_leader = DummyVehicle([0, 0, 0, 0], vehicle_id=0)
    
    def _init_communication(self):
        """Initialize UDP sockets for communication."""
        try:
            # Sending socket
            self.send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.send_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            
            # Receiving socket
            self.recv_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.recv_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.recv_socket.bind(('', self.recv_port))
            self.recv_socket.settimeout(0.1)  # 100ms timeout
            
            self.logger.info(f"Communication initialized - Send port: {self.send_port}, Recv port: {self.recv_port}")
            
        except Exception as e:
            self.logger.error(f"Failed to initialize communication: {e}")
            raise
    
    def _init_leader_control(self):
        """Initialize leader-specific control parameters."""
        self.leader_start_time = None
        # Add any leader-specific initialization here
    
    def update_state(self):
        """Update the vehicle's current state from QLabs."""
        try:
            _, pos, rot, _ = self.qcar.get_world_transform()
            
            with self.lock:
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
    
    def send_data_thread(self):
        """Thread for sending vehicle data to other vehicles."""
        send_interval = 1.0 / self.send_rate
        self.logger.info(f"Starting data transmission thread at {self.send_rate} Hz")
        
        while self.running:
            start_time = time.time()
            
            try:
                with self.lock:
                    # Prepare data packet
                    data = {
                        'vehicle_id': self.vehicle_id,
                        'timestamp': time.time(),
                        'is_leader': self.is_leader,
                        'position': self.current_pos.copy(),
                        'rotation': self.current_rot.copy(),
                        'velocity': self.velocity,
                        'controller_type': self.controller_type
                    }
                
                # Broadcast to all vehicles
                message = json.dumps(data).encode('utf-8')
                for target_vehicle in range(10):  # Assume max 10 vehicles
                    if target_vehicle != self.vehicle_id:
                        target_port = self.base_port + 100 + target_vehicle
                        try:
                            self.send_socket.sendto(message, (self.broadcast_ip, target_port))
                        except Exception as e:
                            # Don't log every failed send attempt
                            pass
                
                self.logger.debug(f"Sent data: pos={self.current_pos}, vel={self.velocity:.3f}")
                
            except Exception as e:
                self.logger.error(f"Error in send_data_thread: {e}")
            
            # Maintain send rate
            elapsed = time.time() - start_time
            sleep_time = max(0, send_interval - elapsed)
            time.sleep(sleep_time)
    
    def receive_data_thread(self):
        """Thread for receiving data from other vehicles."""
        self.logger.info(f"Starting data reception thread at {self.recv_rate} Hz")
        recv_interval = 1.0 / self.recv_rate
        
        while self.running:
            start_time = time.time()
            
            try:
                data, addr = self.recv_socket.recvfrom(4096)
                received_data = json.loads(data.decode('utf-8'))
                
                sender_id = received_data.get('vehicle_id')
                if sender_id is not None and sender_id != self.vehicle_id:
                    with self.lock:
                        self.platoon_data[sender_id] = received_data
                        self.last_update_times[sender_id] = time.time()
                    
                    self.logger.debug(f"Received data from vehicle {sender_id}")
                
            except socket.timeout:
                # Timeout is normal, continue
                pass
            except Exception as e:
                self.logger.error(f"Error in receive_data_thread: {e}")
            
            # Maintain receive rate
            elapsed = time.time() - start_time
            sleep_time = max(0, recv_interval - elapsed)
            time.sleep(sleep_time)
    
    def get_leader_data(self):
        """Get the most recent leader data from platoon."""
        current_time = time.time()
        
        with self.lock:
            for vehicle_id, data in self.platoon_data.items():
                if data.get('is_leader', False):
                    # Check if data is fresh (less than 1 second old)
                    last_update = self.last_update_times.get(vehicle_id, 0)
                    if current_time - last_update < 1.0:
                        return data
        
        return None
    
    def leader_control_logic(self):
        """Leader control logic with time-based velocity profile."""
        if not self.is_leader:
            return
        
        try:
            current_time = time.time()
            if self.leader_start_time is None:
                self.leader_start_time = current_time
            
            elapsed_time = current_time - self.leader_start_time
            
            # Get reference velocity based on road type and time
            if self.config and self.config.get_road_type_name() == "Studio":
                forward_speed = 0.3  # Constant speed for Studio
                steering_angle = 0.0
            else:
                # OpenRoad velocity profile
                if elapsed_time < 5:
                    forward_speed = 2.0
                elif elapsed_time < 10:
                    forward_speed = 0.0
                elif elapsed_time < 15:
                    forward_speed = -0.5
                elif elapsed_time < 20:
                    forward_speed = 1.0
                else:
                    forward_speed = 2.0
                steering_angle = 0.0
            
            # Apply commands to QLabs vehicle
            self.qcar.set_velocity_and_request_state(
                forward=forward_speed,
                turn=steering_angle,
                headlights=False,
                leftTurnSignal=False,
                rightTurnSignal=False,
                brakeSignal=False,
                reverseSignal=False
            )
            
            self.logger.debug(f"Leader control: speed={forward_speed:.3f}, steering={steering_angle:.3f}")
            
        except Exception as e:
            self.logger.error(f"Leader control error: {e}")
    
    def follower_control_logic(self):
        """Follower control logic using CACC/IDM and pure pursuit."""
        if self.is_leader:
            return
        
        try:
            leader_data = self.get_leader_data()
            if leader_data is None:
                # No leader data available, stop
                self.qcar.set_velocity_and_request_state(
                    forward=0.0, turn=0.0, headlights=False,
                    leftTurnSignal=False, rightTurnSignal=False,
                    brakeSignal=False, reverseSignal=False
                )
                self.logger.warning("No leader data available, stopping")
                return
            
            # Extract leader and follower states
            leader_pos = leader_data['position']
            leader_rot = leader_data['rotation']
            leader_velocity = leader_data['velocity']
            
            follower_state = [self.current_pos[0], self.current_pos[1], self.current_rot[2], self.velocity]
            leader_state = [leader_pos[0], leader_pos[1], leader_rot[2], leader_velocity]
            
            # Update dummy leader for controller
            self.dummy_leader.state = leader_state
            self.controller.controller.get_surrounding_vehicles = lambda *args, **kwargs: (None, [self.dummy_leader], None, None)
            
            # Longitudinal control using CACC/IDM
            _, control_input, _ = self.controller.get_optimal_input(
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
            speed_cmd = control_input[0]
            
            # Lateral control using pure pursuit
            target_x = leader_pos[0] - self.lookahead_distance * math.cos(leader_rot[2])
            target_y = leader_pos[1] - self.lookahead_distance * math.sin(leader_rot[2])
            
            dx = target_x - self.current_pos[0]
            dy = target_y - self.current_pos[1]
            target_angle = math.atan2(dy, dx)
            heading_error = wrap_to_pi(target_angle - self.current_rot[2])
            
            steering_cmd = -self.k_steering * heading_error
            steering_cmd = max(-self.max_steering, min(self.max_steering, steering_cmd))
            
            # Apply commands
            self.qcar.set_velocity_and_request_state(
                forward=speed_cmd,
                turn=steering_cmd,
                headlights=False,
                leftTurnSignal=False,
                rightTurnSignal=False,
                brakeSignal=False,
                reverseSignal=False
            )
            
            self.logger.debug(f"Follower control: speed={speed_cmd:.3f}, steering={steering_cmd:.3f}")
            
        except Exception as e:
            self.logger.error(f"Follower control error: {e}")
    
    def control_thread(self):
        """Main control thread that runs the vehicle control loop."""
        control_interval = 1.0 / self.update_rate
        self.logger.info(f"Starting control thread at {self.update_rate} Hz")
        
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
                
            except Exception as e:
                self.logger.error(f"Control thread error: {e}")
            
            # Maintain control rate
            elapsed = time.time() - start_time
            sleep_time = max(0, control_interval - elapsed)
            time.sleep(sleep_time)
    
    def start(self):
        """Start all threads for the vehicle."""
        if self.running:
            self.logger.warning("Vehicle already running")
            return
        
        self.running = True
        
        # Start all threads
        self.threads = [
            threading.Thread(target=self.control_thread, daemon=True, name=f"V{self.vehicle_id}_Control"),
            threading.Thread(target=self.send_data_thread, daemon=True, name=f"V{self.vehicle_id}_Send"),
            threading.Thread(target=self.receive_data_thread, daemon=True, name=f"V{self.vehicle_id}_Recv")
        ]
        
        for thread in self.threads:
            thread.start()
        
        self.logger.info(f"Vehicle {self.vehicle_id} started with {len(self.threads)} threads")
    
    def stop(self):
        """Stop all threads and cleanup."""
        self.running = False
        
        # Wait for threads to finish
        for thread in self.threads:
            if thread.is_alive():
                thread.join(timeout=2.0)
        
        # Stop the vehicle
        try:
            self.qcar.set_velocity_and_request_state(
                forward=0.0, turn=0.0, headlights=False,
                leftTurnSignal=False, rightTurnSignal=False,
                brakeSignal=False, reverseSignal=False
            )
        except Exception as e:
            self.logger.error(f"Error stopping vehicle: {e}")
        
        # Close sockets
        try:
            self.send_socket.close()
            self.recv_socket.close()
        except Exception as e:
            self.logger.error(f"Error closing sockets: {e}")
        
        self.logger.info(f"Vehicle {self.vehicle_id} stopped")
    
    def is_alive(self):
        """Check if the vehicle threads are alive."""
        return self.running and any(thread.is_alive() for thread in self.threads)
    
    def get_state(self):
        """Get current vehicle state."""
        with self.lock:
            return {
                'vehicle_id': self.vehicle_id,
                'position': self.current_pos.copy(),
                'rotation': self.current_rot.copy(),
                'velocity': self.velocity,
                'is_leader': self.is_leader,
                'running': self.running,
                'platoon_size': len(self.platoon_data),
                'last_leader_update': max(self.last_update_times.values()) if self.last_update_times else 0
            }
