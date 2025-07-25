import socket
import threading
import time
import json
import logging
from typing import Dict, List, Any, Optional
from collections import defaultdict


class PlatoonCommunicationManager:
    """
    Centralized communication manager for the platoon that handles
    message routing, reliability, and network coordination.
    """
    
    def __init__(self, base_port: int = 5000, max_vehicles: int = 10):
        """
        Initialize the communication manager.
        
        Args:
            base_port: Base port number for communication
            max_vehicles: Maximum number of vehicles in platoon
        """
        self.base_port = base_port
        self.max_vehicles = max_vehicles
        self.running = False
        
        # Message handling
        self.message_handlers = defaultdict(list)
        self.vehicle_registry = {}  # vehicle_id -> {ip, port, last_seen}
        self.message_queue = []
        self.queue_lock = threading.Lock()
        
        # Statistics
        self.stats = {
            'messages_sent': 0,
            'messages_received': 0,
            'failed_sends': 0,
            'active_vehicles': 0
        }
        
        # Setup logging
        self.logger = logging.getLogger("PlatoonCommManager")
        
        # Network setup
        self.server_socket = None
        self.setup_network()
    
    def setup_network(self):
        """Setup the main communication socket."""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind(('', self.base_port))
            self.server_socket.settimeout(0.1)
            self.logger.info(f"Communication manager listening on port {self.base_port}")
        except Exception as e:
            self.logger.error(f"Failed to setup network: {e}")
            raise
    
    def register_vehicle(self, vehicle_id: int, ip: str, port: int):
        """Register a vehicle in the platoon."""
        self.vehicle_registry[vehicle_id] = {
            'ip': ip,
            'port': port,
            'last_seen': time.time(),
            'active': True
        }
        self.stats['active_vehicles'] = len([v for v in self.vehicle_registry.values() if v['active']])
        self.logger.info(f"Registered vehicle {vehicle_id} at {ip}:{port}")
    
    def unregister_vehicle(self, vehicle_id: int):
        """Unregister a vehicle from the platoon."""
        if vehicle_id in self.vehicle_registry:
            self.vehicle_registry[vehicle_id]['active'] = False
            self.stats['active_vehicles'] = len([v for v in self.vehicle_registry.values() if v['active']])
            self.logger.info(f"Unregistered vehicle {vehicle_id}")
    
    def add_message_handler(self, message_type: str, handler_func):
        """Add a handler function for specific message types."""
        self.message_handlers[message_type].append(handler_func)
    
    def broadcast_message(self, message: Dict[str, Any], sender_id: Optional[int] = None):
        """
        Broadcast a message to all registered vehicles.
        
        Args:
            message: Message dictionary to broadcast
            sender_id: ID of sending vehicle (to avoid echo)
        """
        message['timestamp'] = time.time()
        message_data = json.dumps(message).encode('utf-8')
        
        successful_sends = 0
        for vehicle_id, vehicle_info in self.vehicle_registry.items():
            if not vehicle_info['active'] or vehicle_id == sender_id:
                continue
            
            try:
                target_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                target_socket.sendto(message_data, (vehicle_info['ip'], vehicle_info['port']))
                target_socket.close()
                successful_sends += 1
            except Exception as e:
                self.stats['failed_sends'] += 1
                self.logger.debug(f"Failed to send to vehicle {vehicle_id}: {e}")
        
        self.stats['messages_sent'] += successful_sends
        return successful_sends
    
    def send_to_vehicle(self, vehicle_id: int, message: Dict[str, Any]) -> bool:
        """
        Send a message to a specific vehicle.
        
        Args:
            vehicle_id: Target vehicle ID
            message: Message to send
            
        Returns:
            True if message was sent successfully
        """
        if vehicle_id not in self.vehicle_registry or not self.vehicle_registry[vehicle_id]['active']:
            return False
        
        vehicle_info = self.vehicle_registry[vehicle_id]
        message['timestamp'] = time.time()
        message_data = json.dumps(message).encode('utf-8')
        
        try:
            target_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            target_socket.sendto(message_data, (vehicle_info['ip'], vehicle_info['port']))
            target_socket.close()
            self.stats['messages_sent'] += 1
            return True
        except Exception as e:
            self.stats['failed_sends'] += 1
            self.logger.debug(f"Failed to send to vehicle {vehicle_id}: {e}")
            return False
    
    def handle_incoming_message(self, data: bytes, addr: tuple):
        """Process incoming messages from vehicles."""
        try:
            message = json.loads(data.decode('utf-8'))
            message_type = message.get('type', 'unknown')
            sender_id = message.get('vehicle_id', -1)
            
            # Update vehicle registry
            if sender_id >= 0:
                if sender_id not in self.vehicle_registry:
                    self.register_vehicle(sender_id, addr[0], addr[1])
                else:
                    self.vehicle_registry[sender_id]['last_seen'] = time.time()
            
            # Handle different message types
            if message_type == 'state_update':
                self.handle_state_update(message)
            elif message_type == 'heartbeat':
                self.handle_heartbeat(message)
            elif message_type == 'emergency_stop':
                self.handle_emergency_stop(message)
            elif message_type == 'registration':
                self.handle_registration(message, addr)
            
            # Call registered handlers
            for handler in self.message_handlers[message_type]:
                try:
                    handler(message, addr)
                except Exception as e:
                    self.logger.error(f"Error in message handler: {e}")
            
            self.stats['messages_received'] += 1
            
        except Exception as e:
            self.logger.error(f"Error handling incoming message: {e}")
    
    def handle_state_update(self, message: Dict[str, Any]):
        """Handle vehicle state update messages."""
        vehicle_id = message.get('vehicle_id')
        if vehicle_id is not None:
            # Relay to other vehicles (excluding sender)
            self.broadcast_message(message, sender_id=vehicle_id)
    
    def handle_heartbeat(self, message: Dict[str, Any]):
        """Handle heartbeat messages from vehicles."""
        vehicle_id = message.get('vehicle_id')
        if vehicle_id in self.vehicle_registry:
            self.vehicle_registry[vehicle_id]['last_seen'] = time.time()
    
    def handle_emergency_stop(self, message: Dict[str, Any]):
        """Handle emergency stop messages."""
        self.logger.warning(f"Emergency stop received from vehicle {message.get('vehicle_id')}")
        # Broadcast emergency stop to all vehicles
        emergency_msg = {
            'type': 'emergency_stop',
            'reason': message.get('reason', 'Unknown'),
            'source_vehicle': message.get('vehicle_id')
        }
        self.broadcast_message(emergency_msg)
    
    def handle_registration(self, message: Dict[str, Any], addr: tuple):
        """Handle vehicle registration messages."""
        vehicle_id = message.get('vehicle_id')
        port = message.get('port', addr[1])
        if vehicle_id is not None:
            self.register_vehicle(vehicle_id, addr[0], port)
    
    def cleanup_stale_vehicles(self):
        """Remove vehicles that haven't been seen recently."""
        current_time = time.time()
        stale_timeout = 10.0  # 10 seconds
        
        stale_vehicles = []
        for vehicle_id, info in self.vehicle_registry.items():
            if current_time - info['last_seen'] > stale_timeout and info['active']:
                stale_vehicles.append(vehicle_id)
        
        for vehicle_id in stale_vehicles:
            self.unregister_vehicle(vehicle_id)
            self.logger.warning(f"Vehicle {vehicle_id} marked as inactive (stale)")
    
    def communication_loop(self):
        """Main communication loop."""
        self.logger.info("Starting communication loop")
        
        while self.running:
            try:
                # Receive messages
                data, addr = self.server_socket.recvfrom(4096)
                self.handle_incoming_message(data, addr)
                
            except socket.timeout:
                # Timeout is normal, use it for cleanup
                self.cleanup_stale_vehicles()
                
            except Exception as e:
                self.logger.error(f"Error in communication loop: {e}")
    
    def start(self):
        """Start the communication manager."""
        if self.running:
            return
        
        self.running = True
        self.comm_thread = threading.Thread(target=self.communication_loop, daemon=True)
        self.comm_thread.start()
        self.logger.info("Communication manager started")
    
    def stop(self):
        """Stop the communication manager."""
        self.running = False
        if hasattr(self, 'comm_thread'):
            self.comm_thread.join(timeout=2.0)
        
        if self.server_socket:
            self.server_socket.close()
        
        self.logger.info("Communication manager stopped")
    
    def get_stats(self) -> Dict[str, Any]:
        """Get communication statistics."""
        return {
            **self.stats,
            'registered_vehicles': len(self.vehicle_registry),
            'active_vehicles': len([v for v in self.vehicle_registry.values() if v['active']])
        }
    
    def get_vehicle_list(self) -> List[int]:
        """Get list of active vehicle IDs."""
        return [vid for vid, info in self.vehicle_registry.items() if info['active']]
