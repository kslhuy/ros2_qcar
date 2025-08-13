import socket
import json as ujson
import time
import logging
from typing import Dict, Any, Tuple, Optional
import threading

# Import performance monitoring
try:
    from performance_monitor import perf_monitor
    PERFORMANCE_MONITORING = True
except ImportError:
    PERFORMANCE_MONITORING = False
    perf_monitor = None

class CommHandler:
    """
    Handles UDP communication for vehicle fleet coordination.
    
    This class manages:
    - Sending vehicle state updates with reliable delivery (ACK mechanism)
    - Receiving messages from other vehicles (state updates, heartbeats)
    - ACK processing for reliable message delivery
    - Thread-safe operations for concurrent communication
    
    Communication Protocol:
    - State messages: Vehicle position, rotation, velocity data
    - ACK messages: Acknowledgment of received state messages
    - Heartbeat messages: Keep-alive signals between vehicles
    """
    
    # Communication timing constants
    STATE_SEND_PERIOD = 0.01        # Send state every 50ms (20Hz)
    RECEIVE_PERIOD = 0.01            # Receive loop period 50ms (20Hz)
    ACK_TIMEOUT = 0.5               # Wait 500ms for ACK response
    RECV_TIMEOUT = 0.01             # Socket receive timeout 10ms
    MAX_RETRIES = 3                 # Maximum retry attempts for failed sends
    HEARTBEAT_TIMEOUT = 2.0         # Heartbeat timeout threshold
    
    def __init__(self, vehicle_id: int, target_ip: str, send_port: int, recv_port: int, 
                 ack_port: int, logger: logging.LoggerAdapter, running_flag, vehicle):
        """
        Initialize communication handler for a vehicle.
        
        Args:
            vehicle_id: Unique identifier for this vehicle
            target_ip: IP address to send messages to (broadcast or specific vehicle)
            send_port: Port for sending state messages
            recv_port: Port for receiving messages from other vehicles
            ack_port: Port for receiving ACK confirmations
            logger: Logger instance for debugging and monitoring
            running_flag: Shared threading event to control operation
            vehicle: Reference to the Vehicle instance for state access
        """
        # Vehicle identification and network configuration
        self.vehicle_id = vehicle_id
        self.target_ip = target_ip
        self.send_port = send_port
        self.recv_port = recv_port
        self.ack_port = ack_port
        self.logger = logger
        
        # Initialize UDP sockets for different communication purposes
        self._setup_sockets()
        
        # Message sequencing and synchronization
        self.sequence_number = 0
        self.last_heartbeat_time = time.time()
        self.heartbeat_timeout = self.HEARTBEAT_TIMEOUT
        self.heartbeat = True
        
        # Thread synchronization
        self.lock = threading.Lock()
        self.running = running_flag  # Reference to Vehicle's running flag
        self.vehicle = vehicle       # Reference to Vehicle instance
        
        self.logger.info(f"CommHandler initialized for Vehicle {self.vehicle_id}")
        
    def _setup_sockets(self):
        """
        Initialize and configure all UDP sockets for communication.
        
        Creates four sockets:
        - send_sock: For sending state messages to other vehicles
        - recv_sock: For receiving messages from other vehicles
        - send_ack_sock: For sending ACK responses
        - ack_sock: For receiving ACK confirmations
        """
        try:
            # Socket for sending state messages
            self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            
            # Socket for receiving messages from other vehicles
            self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.recv_sock.bind(('0.0.0.0', self.recv_port))
            self.recv_sock.settimeout(self.RECV_TIMEOUT)
            
            # Socket for sending ACK responses
            self.send_ack_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            
            # Socket for receiving ACK confirmations
            self.ack_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.ack_sock.settimeout(self.ACK_TIMEOUT)
            self.ack_sock.bind(('0.0.0.0', self.ack_port))
            
            self.logger.info(f"Sockets configured - recv_port: {self.recv_port}, ack_port: {self.ack_port}")
            
        except Exception as e:
            self.logger.error(f"Failed to setup sockets: {e}")
            raise

    def _create_state_message(self) -> Dict[str, Any]:
        """
        Create a state message with current vehicle information.
        
        Returns:
            Dictionary containing vehicle state data for transmission
        """
        with self.lock:
            # Gather current vehicle state data
            current_pos = self.vehicle.current_pos
            current_rot = self.vehicle.current_rot
            velocity = self.vehicle.velocity
            
            # Use GPS-synchronized time if available, fallback to system time
            try:
                synced_time = self.vehicle.gps_sync.get_synced_time()
            except AttributeError:
                synced_time = time.time()  # Fallback if GPS sync not available
            
            # Create structured message
            state_data = {
                'type': 'state',
                'seq': self.sequence_number,
                'id': self.vehicle_id,
                'pos': current_pos,
                'rot': current_rot,
                'v': velocity,
                'timestamp': synced_time,
                'ack_port': self.ack_port
            }
            
        return state_data
    
    def _send_state_message(self, data: Dict[str, Any]) -> bool:
        """
        Send a state message via UDP socket.
        
        Args:
            data: State message data to send
            
        Returns:
            True if message sent successfully, False otherwise
        """
        try:
            message_json = ujson.dumps(data).encode()
            self.send_sock.sendto(message_json, (self.target_ip, self.send_port))
            
            self.logger.info(f"SENT: Seq: {self.sequence_number}, "
                           f"Pos: {data['pos']}, V: {data['v']:.3f}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to send state message: {e}")
            return False
    
    def _wait_for_ack(self) -> bool:
        """
        Wait for ACK response after sending a state message.
        
        Returns:
            True if valid ACK received, False on timeout or invalid ACK
        """
        try:
            self.ack_sock.settimeout(self.ACK_TIMEOUT)
            ack_data, addr = self.ack_sock.recvfrom(1024)
            ack = ujson.loads(ack_data.decode())
            
            self.logger.debug(f"ACK received on port {self.ack_port} from {addr}")
            self.logger.debug(f"ACK content: ack_id={ack.get('ack_id')}, "
                            f"ack_seq={ack.get('ack_seq')}, expected_seq={self.sequence_number}")
            
            # Validate ACK message
            if (ack.get('type') == 'ack' and 
                ack.get('ack_seq') == self.sequence_number and 
                ack.get('ack_id') != self.vehicle_id):  # Don't ACK our own messages
                
                # Increment sequence number for next message
                with self.lock:
                    self.sequence_number += 1
                    
                self.logger.info(f"Valid ACK received for seq: {self.sequence_number - 1} from {addr}")
                return True
            else:
                self.logger.debug(f"ACK discarded - invalid or unexpected: {ack}")
                return False
                
        except socket.timeout:
            self.logger.warning(f"ACK timeout for seq: {self.sequence_number}")
            return False
        except Exception as e:
            self.logger.error(f"ACK processing error: {e}")
            return False

    def send_state(self):
        """
        Main state transmission loop - runs in dedicated thread.
        
        Continuously sends vehicle state updates with reliable delivery:
        1. Create state message with current vehicle data
        2. Send message via UDP
        3. Wait for ACK confirmation
        4. Retry up to MAX_RETRIES times if no ACK received
        5. Maintain consistent transmission rate
        """
        self.logger.info(f"Starting state transmission thread for Vehicle {self.vehicle_id}")
        
        try:
            while self.running.is_set():
                self.logger.debug(f"Vehicle {self.vehicle_id} send_state active, seq: {self.sequence_number}")
                
                start_time = time.time()
                retries = 0
                ack_received = False
                
                # Retry loop for reliable delivery
                while retries < self.MAX_RETRIES and not ack_received and self.running.is_set():
                    try:
                        # Step 1: Create state message
                        state_data = self._create_state_message()
                        
                        # Step 2: Send state message
                        if self._send_state_message(state_data):
                            # Step 3: Wait for ACK confirmation
                            ack_received = self._wait_for_ack()
                        
                        if not ack_received:
                            retries += 1
                            if retries < self.MAX_RETRIES:
                                self.logger.warning(f"Retry {retries}/{self.MAX_RETRIES} for seq: {self.sequence_number}")
                                time.sleep(0.1)  # Brief delay before retry
                            
                    except Exception as e:
                        self.logger.error(f"State transmission error: {e}")
                        retries += 1
                        time.sleep(0.1)
                
                # Log final result for this transmission cycle
                if not ack_received:
                    self.logger.error(f"Failed to receive ACK for seq: {self.sequence_number} after {self.MAX_RETRIES} retries")
                
                # Maintain consistent transmission rate
                elapsed = time.time() - start_time
                sleep_time = max(0, self.STATE_SEND_PERIOD - elapsed)
                time.sleep(sleep_time)
                
        except Exception as e:
            self.logger.error(f"State transmission thread crashed: {e}")
        finally:
            self.logger.info(f"Vehicle {self.vehicle_id} state transmission thread exited")

    def _send_ack_response(self, seq: int, sender_addr: Tuple[str, int], sender_ack_port: int):
        """
        Send ACK response to acknowledge received message.
        
        Args:
            seq: Sequence number to acknowledge
            sender_addr: Address tuple (IP, port) of message sender
            sender_ack_port: Port where sender expects ACK response
        """
        try:
            ack_message = {
                'type': 'ack',
                'ack_seq': seq,
                'ack_id': self.vehicle_id
            }
            
            ack_json = ujson.dumps(ack_message).encode()
            self.send_ack_sock.sendto(ack_json, (sender_addr[0], sender_ack_port))
            
            self.logger.info(f"SENT: ACK for seq: {seq} to {sender_addr[0]}:{sender_ack_port}")
            
        except Exception as e:
            self.logger.error(f"Failed to send ACK response: {e}, sender: {sender_addr}")
    
    def _handle_state_message(self, message: Dict[str, Any], sender_addr: Tuple[str, int]):
        """
        Process received state message from another vehicle.
        
        Args:
            message: Decoded state message data
            sender_addr: Address tuple (IP, port) of message sender
        """
        sender_id = message.get('id')
        seq = message.get('seq', -1)
        
        # Ignore messages from ourselves
        if sender_id == self.vehicle_id:
            return
            
        self.logger.info(f"RECEIVED STATE: Seq: {seq}, Sender ID: {sender_id}, "
                        f"Pos: {message.get('pos')}, V: {message.get('v', 0.0):.3f}")
        
        # Send ACK response
        sender_ack_port = message.get('ack_port')
        if sender_ack_port:
            self._send_ack_response(seq, sender_addr, sender_ack_port)
        
        # Process state through Vehicle's enhanced state management
        try:
            # Call the Vehicle's process_received_state method which handles StateQueue validation
            self.vehicle.process_received_state(message)
        except Exception as e:
            self.logger.error(f"Error processing received state through Vehicle: {e}")
            
            # Fallback: Update follower's leader state directly (old method)
            if not self.vehicle.is_leader:
                with self.lock:
                    self.vehicle.leader_state = message
                    self.logger.info(f"Fallback: Updated leader_state from Vehicle {sender_id}")
    
    def _handle_heartbeat_message(self, message: Dict[str, Any]):
        """
        Process received heartbeat message.
        
        Args:
            message: Decoded heartbeat message data
        """
        sender_id = message.get('id')
        self.last_heartbeat_time = time.time()
        self.heartbeat = True
        self.logger.info(f"RECEIVED HEARTBEAT from Vehicle {sender_id}")
    
    def _handle_ack_message(self, message: Dict[str, Any]):
        """
        Process received ACK message.
        
        Args:
            message: Decoded ACK message data
            
        Note: ACK messages are primarily handled in _wait_for_ack() method.
        This method is for any additional ACK processing if needed.
        """
        # ACK messages are handled in the send_state method's _wait_for_ack call
        self.logger.debug(f"ACK message received: {message}")
    
    def _process_received_message(self, data: bytes, sender_addr: Tuple[str, int]):
        """
        Decode and route received message to appropriate handler.
        
        Args:
            data: Raw message bytes received from socket
            sender_addr: Address tuple (IP, port) of message sender
        """
        try:
            # Start JSON decode timing
            if PERFORMANCE_MONITORING:
                json_start = perf_monitor.start_timing()
            
            # Fast JSON decode
            message = ujson.loads(data.decode())
            
            # End JSON decode timing
            if PERFORMANCE_MONITORING:
                perf_monitor.end_timing(json_start, "json_decode")
            
            msg_type = message.get('type', '')
            sender_id = message.get('id')
            
            # Quick type check to avoid unnecessary processing
            if sender_id == self.vehicle_id:
                return  # Ignore messages from ourselves
            
            # Route to appropriate message handler (optimized order by frequency)
            if msg_type == 'state':
                self._handle_state_message_optimized(message, sender_addr)
            elif msg_type == 'ack':
                self._handle_ack_message(message)
            elif msg_type == 'heartbeat':
                self._handle_heartbeat_message(message)
            else:
                self.logger.warning(f"Unknown message type received: {msg_type} from {sender_addr}")
                
        except Exception as e:
            self.logger.error(f"Failed to process received message: {e}, sender: {sender_addr}")
    
    def _handle_state_message_optimized(self, message: Dict[str, Any], sender_addr: Tuple[str, int]):
        """
        Optimized version of state message handling.
        
        Args:
            message: Decoded state message data
            sender_addr: Address tuple (IP, port) of message sender
        """
        # Start performance timing
        if PERFORMANCE_MONITORING:
            process_start = perf_monitor.start_timing()
        
        # Extract essential data quickly
        sender_id = message.get('id')
        seq = message.get('seq', -1)
        
        # Log with minimal formatting for performance
        self.logger.info(f"RECEIVED STATE: Seq: {seq}, Sender ID: {sender_id}, "
                        f"Pos: {message.get('pos')}, V: {message.get('v', 0.0):.3f}")
        
        # Send ACK response (non-blocking)
        sender_ack_port = message.get('ack_port')
        if sender_ack_port:
            self._send_ack_response(seq, sender_addr, sender_ack_port)
        
        # Process state through Vehicle's enhanced state management
        # This is the potentially slow part - do it last
        try:
            if PERFORMANCE_MONITORING:
                validation_start = perf_monitor.start_timing()
            
            self.vehicle.process_received_state(message)
            
            if PERFORMANCE_MONITORING:
                perf_monitor.end_timing(validation_start, "validation")
        except Exception as e:
            self.logger.error(f"Error processing received state through Vehicle: {e}")
            
            # Fallback: Update follower's leader state directly (old method)
            if not self.vehicle.is_leader:
                with self.lock:
                    self.vehicle.leader_state = message
                    self.logger.info(f"Fallback: Updated leader_state from Vehicle {sender_id}")
        
        # End performance timing
        if PERFORMANCE_MONITORING:
            perf_monitor.end_timing(process_start, "processing")

    def receive_messages(self):
        """
        Main message reception loop - runs in dedicated thread.
        
        Continuously listens for incoming messages:
        1. Receive UDP packets on the designated port
        2. Decode and validate message format
        3. Route messages to appropriate handlers based on type
        4. Send ACK responses for state messages
        5. Update vehicle state for follower vehicles
        """
        self.logger.info(f"Starting message reception thread for Vehicle {self.vehicle_id}")
        
        while self.running.is_set():
            start_time = time.time()
            
            try:
                # Inner loop for more responsive shutdown checking
                while time.time() - start_time < 0.09 and self.running.is_set():
                    try:
                        # Attempt to receive message with timeout
                        data, sender_addr = self.recv_sock.recvfrom(1024)
                        
                        # self.logger.debug(f"Vehicle {self.vehicle_id} received data from {sender_addr}")
                        
                        # Process the received message
                        self._process_received_message(data, sender_addr)
                        
                    except socket.timeout:
                        # Timeout is expected - allows for responsive shutdown
                        break
                    except Exception as e:
                        self.logger.error(f"Message reception error: {e}")
                        break
                        
            except Exception as e:
                self.logger.error(f"Reception loop error: {e}")
            
            # Maintain consistent loop timing
            elapsed = time.time() - start_time
            sleep_time = max(0, self.RECEIVE_PERIOD - elapsed)
            time.sleep(sleep_time)
            
        self.logger.info(f"Message reception thread terminated for Vehicle {self.vehicle_id}")

    def is_heartbeat_alive(self) -> bool:
        """
        Check if heartbeat signal is still active.
        
        Returns:
            True if recent heartbeat received, False if timeout exceeded
        """
        time_since_heartbeat = time.time() - self.last_heartbeat_time
        is_alive = time_since_heartbeat < self.heartbeat_timeout
        
        if not is_alive:
            self.logger.warning(f"Heartbeat timeout: {time_since_heartbeat:.2f}s since last heartbeat")
            
        return is_alive
    
    def get_communication_stats(self) -> Dict[str, Any]:
        """
        Get current communication statistics for monitoring.
        
        Returns:
            Dictionary with communication status and statistics
        """
        return {
            'vehicle_id': self.vehicle_id,
            'sequence_number': self.sequence_number,
            'heartbeat_active': self.heartbeat,
            'last_heartbeat_time': self.last_heartbeat_time,
            'time_since_heartbeat': time.time() - self.last_heartbeat_time,
            'is_running': self.running.is_set(),
            'target_ip': self.target_ip,
            'ports': {
                'send': self.send_port,
                'receive': self.recv_port,
                'ack': self.ack_port
            }
        }

    def cleanup(self):
        """
        Clean shutdown of communication handler.
        
        Safely closes all UDP sockets and logs cleanup status.
        Should be called when shutting down the vehicle or communication system.
        """
        self.logger.info(f"Starting cleanup for Vehicle {self.vehicle_id} CommHandler")
        
        sockets = [
            ('send_sock', self.send_sock),
            ('recv_sock', self.recv_sock), 
            ('send_ack_sock', self.send_ack_sock),
            ('ack_sock', self.ack_sock)
        ]
        
        for socket_name, sock in sockets:
            try:
                if sock:
                    sock.close()
                    self.logger.debug(f"Closed {socket_name}")
            except Exception as e:
                self.logger.error(f"Error closing {socket_name}: {e}")
        
        self.logger.info(f"CommHandler cleanup completed for Vehicle {self.vehicle_id}")
