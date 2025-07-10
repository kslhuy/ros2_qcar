# JSON
# Sync with GPS Server: Make GPS synchronization between vehicles consistent and more realistic
# Fail-safe mechanism: Log warnings if no heartbeats are received, with potential to trigger a stop in update_movement
# ACKs: Implement a mechanism where the receiver sends an acknowledgment for each state packet, and the sender retries if no ACK is received within a timeout.
# Heartbeats: Send periodic heartbeat messages to detect vehicle disconnections, enabling fail-safe actions if no heartbeats are received.
import socket
import ujson  # Faster JSON library
import threading
import math
import time
import random
import logging
from typing import Dict, List, Any

logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] V%(vehicle_id)s: %(message)s')
logger = logging.getLogger(__name__)

class GPSSync:
    """GPS-like time sync via centralized time server."""
    def __init__(self, gps_server_ip='127.0.0.1', gps_server_port=8001):
        self.gps_time_offset = 0
        self.last_sync_time = time.time()
        self.gps_server_ip = gps_server_ip
        self.gps_server_port = gps_server_port
        self.logger = logging.LoggerAdapter(logger, {'vehicle_id': 'GPS'})
        self.last_valid_offset = 0  # Store last valid offset

    def request_gps_time(self):
        """Fetch GPS time from external server."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(0.2)
        try:
            sock.sendto(b"time_request", (self.gps_server_ip, self.gps_server_port))
            data, _ = sock.recvfrom(1024)
            gps_time = ujson.loads(data.decode())
            # Simulate GPS jitter (±10ms)
            gps_time += random.uniform(-0.01, 0.01)
            return gps_time
        except Exception as e:
            self.logger.error(f"Error contacting GPS server: {e}")
            return time.time() + self.last_valid_offset  # Use last valid offset
        finally:
            sock.close()

    def sync_with_gps(self):
        gps_time = self.request_gps_time()
        local_time = time.time()
        self.gps_time_offset = gps_time - local_time
        self.last_valid_offset = self.gps_time_offset  # Update last valid offset
        self.last_sync_time = local_time
        self.logger.info(f"GPS Time: {gps_time:.3f}, Local Time: {local_time:.3f}, Offset: {self.gps_time_offset:.3f} sec")

    def get_synced_time(self):
        return time.time() + self.gps_time_offset

class Vehicle:
    """Represents a vehicle in a platoon, supporting leader or follower roles."""
    def __init__(self, qcar: Any, idm_controller: Any, vehicle_id: int, is_leader: bool = False,
                 max_steering: float = 0.6, ip: str = '127.0.0.1', send_port: int = 5050,
                 recv_port: int = 5005, ack_port: int = 5051):
        self.qcar = qcar
        self.idm = idm_controller
        self.vehicle_id = vehicle_id
        self.is_leader = is_leader
        self.max_steering = max_steering
        self.gps_sync = GPSSync()
        self.running = False
        self.thread = None
        self.logger = logging.LoggerAdapter(logger, {'vehicle_id': vehicle_id})

        # Communication setup
        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock.bind(('0.0.0.0', recv_port))
        self.recv_sock.settimeout(0.01)  # 10ms timeout
        self.send_ack_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ack_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        self.ack_sock.settimeout(0.2)  # 200ms timeout for ACK
        self.target_ip = ip
        self.send_port = send_port
        self.recv_port = recv_port
        self.ack_port = ack_port + vehicle_id  # Unique ACK port per vehicle
        self.ack_sock.bind(('0.0.0.0', self.ack_port))
        self.leader_state = {'pos': [-1.205, -0.83, 0.005], 'rot': [0, 0, -44.7], 'v': 0.3}
        self.last_seq = -1
        self.sequence_number = 0
        self.last_heartbeat_time = time.time()
        self.heartbeat_timeout = 2.0  # Timeout after 2 seconds
        self.lock = threading.Lock()

        # State for sending
        self.current_pos = [0, 0, 0]
        self.current_rot = [0, 0, 0]
        self.velocity = 0.3  # Initial velocity
        self.last_update_time = time.time()
        self.prev_pos = None
        self.prev_time = None
        self.last_sync_attempt = time.time()
        self.sync_interval = 5.0  # seconds
        
        self.heartbeat = True

    def send_state(self):
        """Sends vehicle state at a fixed 10 Hz with ACKs and heartbeats."""
        target_period = 0.1  # 10 Hz
        heartbeat_interval = 1.0  # Heartbeat every 1 second
        max_retries = 3
        while self.running:
            start_time = time.time()
            # Send state
            retries = 0
            ack_received = False
            while retries < max_retries and not ack_received and self.running:
                try:
                    with self.lock:
                        data = {
                            'type': 'state',
                            'seq': self.sequence_number,
                            'id': self.vehicle_id,
                            'pos': self.current_pos,
                            'rot': self.current_rot,
                            'v': self.velocity,
                            'timestamp': self.gps_sync.get_synced_time()
                        }
                    # network_delay = random.uniform(0.05, 0.15)  # 50-150ms delay
                    # time.sleep(network_delay)
                    self.send_sock.sendto(ujson.dumps(data).encode(), (self.target_ip, self.send_port))
                    self.logger.info(f"SENT: Seq: {self.sequence_number}, Pos: {self.current_pos}, V: {self.velocity:.3f}")

                    # Wait for ACK
                    try:
                        print("pass1")
                        ack_data, _ = self.ack_sock.recvfrom(1024)
                        print("vehicle_id:", self.vehicle_id)
                        ack = ujson.loads(ack_data.decode())
                        print("ack id ",ack.get('ack_id'))
                        print("ack decoded:", ack)
                        # if ack.get('type') == 'ack' and ack.get('ack_seq') == self.sequence_number and ack.get('ack_id') != self.vehicle_id:
                        if ack.get('type') == 'ack' and ack.get('ack_seq') == self.sequence_number:
                            print("pass2")
                            ack_received = True
                            with self.lock:
                                self.sequence_number += 1
                            self.logger.info(f"ACK received for seq: {self.sequence_number - 1}")
                    except socket.timeout:
                        print("pass3")
                        retries += 1
                        self.logger.warning(f"ACK timeout for seq: {self.sequence_number}, retry {retries}/{max_retries}")
                    except Exception as e:
                        self.logger.error(f"ACK ERROR: {e}")
                        retries += 1
                except Exception as e:
                    self.logger.error(f"SEND ERROR: {e}")
                    retries += 1
                    time.sleep(0.1)
                    continue

            if not ack_received:
                self.logger.error(f"Failed to receive ACK for seq: {self.sequence_number} after {max_retries} retries")

            # Send heartbeat if interval elapsed
            if time.time() - self.last_heartbeat_time >= heartbeat_interval:
                try:
                    heartbeat = {
                        'type': 'heartbeat',
                        'id': self.vehicle_id,
                        'timestamp': self.gps_sync.get_synced_time()
                    }
                    self.send_sock.sendto(ujson.dumps(heartbeat).encode(), (self.target_ip, self.send_port))
                    self.last_heartbeat_time = time.time()
                    self.logger.info("SENT: Heartbeat")
                except Exception as e:
                    self.logger.error(f"HEARTBEAT SEND ERROR: {e}")

            elapsed = time.time() - start_time
            sleep_time = max(0, target_period - elapsed)
            time.sleep(sleep_time)
    
    def receive_state(self):
        """Receives state, heartbeats, or ACKs from other vehicles at a fixed 10 Hz."""
        target_period = 0.1  # 10 Hz
        while self.running:
            start_time = time.time()
            try:
                data, addr = self.recv_sock.recvfrom(1024)
                incoming = ujson.loads(data.decode())

                msg_type = incoming.get('type', '')

                if msg_type == 'state':
                    sender_id = incoming.get('id', -1)
                    if sender_id != self.vehicle_id and self.is_leader == False:
                        with self.lock:
                            seq = incoming.get('seq', -1)
                            print(f"RECEIVED: Seq: {seq}, Sender ID: {sender_id}, Pos: {incoming['pos']}, V: {incoming['v']:.3f}")
                            if self.last_seq != -1:
                                missed = seq - self.last_seq - 1
                                if missed > 0:
                                    self.logger.warning(f"Missed {missed} state packets (seq {self.last_seq+1} to {seq-1})")
                            else:
                                self.logger.info(f"RECEIVED: Seq: {seq} (initial packet)")

                            self.last_seq = seq
                            self.leader_state = incoming
                            # self.logger.info(f"RECEIVED: Seq: {seq}, Leader state: pos={self.leader_state['pos']}")

                            # Send ACK
                            try:
                                ack = {'type': 'ack', 'ack_seq': seq, 'ack_id': self.vehicle_id}
                                sender_ack_port = 5051 + sender_id
                                self.send_ack_sock.sendto(ujson.dumps(ack).encode(), (addr[0], sender_ack_port))
                                self.logger.info(f"SENT: ACK for seq: {seq}")
                            except Exception as e:
                                self.logger.error(f"ACK SEND ERROR: {e}")

                elif msg_type == 'heartbeat':
                    self.last_heartbeat_time = time.time()
                    self.heartbeat = True
                    sender_id = incoming.get('id', '?')
                    self.logger.info(f"RECEIVED: Heartbeat from V{sender_id}")

                elif msg_type == 'ack':
                    # ACKs are handled in send_state(), so we just ignore them here
                    pass

                else:
                    self.logger.warning(f"Unknown message type received: {msg_type}")

            except socket.timeout:
                if time.time() - self.last_heartbeat_time > self.heartbeat_timeout:
                    self.heartbeat = False
                    self.logger.error("No heartbeat received for over 2 seconds, assuming leader failure")

            except Exception as e:
                elapsed = time.time() - start_time
                self.logger.error(f"RECEIVE ERROR: {e}, Elapsed: {elapsed:.6f} s")

            elapsed = time.time() - start_time
            sleep_time = max(0, target_period - elapsed)
            time.sleep(sleep_time)


    def wrap_to_pi(self, angle: float) -> float:
        """Wraps angle to [-pi, pi]."""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def update_movement(self):
        """Updates vehicle movement based on role (leader or follower)."""
        if self.is_leader:
            try:
                _, pos_leader, rot_leader, _ = self.qcar.get_world_transform()
                self.current_pos = pos_leader
                self.current_rot = rot_leader
            except Exception as e:
                self.logger.error(f"READ ERROR: {e}")
        else:
            if not self.heartbeat:
                self.logger.error("Leader failure detected, stopping vehicle")
                try:
                    self.qcar.set_velocity_and_request_state(
                        forward=0.0, turn=0.0, headlights=False, leftTurnSignal=False,
                        rightTurnSignal=False, brakeSignal=False, reverseSignal=False
                    )
                except Exception as e:
                    self.logger.error(f"CONTROL ERROR: {e}")
                return
            
            self.logger.info("--------Follower update---------")
            try:
                start_time = time.time()
                _, pos_follower, rot_follower, _ = self.qcar.get_world_transform()
                elapsed = time.time() - start_time
            except Exception as e:
                self.logger.error(f"READ ERROR: {e}")
                try:
                    self.qcar.set_velocity_and_request_state(
                        forward=0.0, turn=0.0, headlights=False, leftTurnSignal=False,
                        rightTurnSignal=False, brakeSignal=False, reverseSignal=False
                    )
                except Exception as e:
                    self.logger.error(f"CONTROL ERROR: {e}")
                return

            # Update state for sending
            self.current_pos = pos_follower
            self.current_rot = rot_follower

            # Calculate follower velocity
            current_time = time.time()
            if self.prev_pos is not None and self.prev_time is not None:
                dx = pos_follower[0] - self.prev_pos[0]
                dy = pos_follower[1] - self.prev_pos[1]
                distance = math.sqrt(dx**2 + dy**2)
                dt = current_time - self.prev_time
                self.velocity = distance / dt if dt > 1e-6 else 0.0
            else:
                self.velocity = 0.0
            self.prev_pos = pos_follower
            self.prev_time = current_time

            # Pure pursuit for steering
            pos_leader = self.leader_state['pos']
            rot_leader = self.leader_state['rot']
            self.logger.info(f"Leader pos: {pos_leader}")
            v_leader = self.leader_state['v']
            lookahead_distance = 0.4
            target_x = pos_leader[0] - lookahead_distance * math.cos(rot_leader[2])
            target_y = pos_leader[1] - lookahead_distance * math.sin(rot_leader[2])
            dx = target_x - pos_follower[0]
            dy = target_y - pos_follower[1]
            follower_heading = rot_follower[2]
            target_angle = math.atan2(dy, dx)
            heading_error = self.wrap_to_pi(target_angle - follower_heading)
            steering_cmd = -2.0 * heading_error
            steering_cmd = max(-self.max_steering, min(self.max_steering, steering_cmd))

            # CACC for speed
            v_follower = self.velocity
            follower_state = [pos_follower[0], pos_follower[1], follower_heading, v_follower]
            class DummyVehicle:
                def __init__(self, state, vehicle_number=0):
                    self.state = state
                    self.vehicle_number = vehicle_number
            leader_state = [pos_leader[0], pos_leader[1], rot_leader[2], v_leader]
            dummy_leader = DummyVehicle(leader_state, vehicle_number=0)
            self.idm.controller.get_surrounding_vehicles = lambda *args, **kwargs: (None, [dummy_leader], None, None)

            try:
                start_time = time.time()
                _, input_u, _ = self.idm.get_optimal_input(
                    host_car_id=self.vehicle_id, state=follower_state, last_input=None,
                    lane_id=None, input_log=None, initial_lane_id=None, direction_flag=None,
                    type_state="true", acc_flag=0
                )
                speed_cmd = max(0, input_u[0])
                elapsed = time.time() - start_time
            except Exception as e:
                self.logger.error(f"IDM/CACC ERROR: {e}")
                speed_cmd = 0.0

            try:
                start_time = time.time()
                self.qcar.set_velocity_and_request_state(
                    forward=speed_cmd, turn=steering_cmd, headlights=False,
                    leftTurnSignal=False, rightTurnSignal=False, brakeSignal=False, reverseSignal=False
                )
                elapsed = time.time() - start_time
            except Exception as e:
                self.logger.error(f"CONTROL ERROR: {e}")

    def run(self):
        """Runs the vehicle control loop at 100 Hz."""
        self.logger.info(f"Starting run loop, is_leader: {self.is_leader}")
        self.running = True
        self.gps_sync.sync_with_gps()
        target_period = 0.1  # 100 Hz
        while self.running:
            start_time = time.time()
            # Periodic GPS synchronization
            if start_time - self.last_sync_attempt >= self.sync_interval:
                self.gps_sync.sync_with_gps()
                self.last_sync_attempt = start_time
            self.update_movement()
            elapsed = time.time() - start_time
            sleep_time = max(0, target_period - elapsed)
            self.logger.info(f"RUN: Sleep time: {sleep_time:.6f} s")
            time.sleep(sleep_time)

    def start(self):
        """Starts the vehicle threads."""
        self.logger.info(f"Starting vehicle, is_leader: {self.is_leader}")
        if self.thread is None or not self.thread.is_alive():
            self.running = True
            threading.Thread(target=self.send_state, daemon=True).start()
            threading.Thread(target=self.receive_state, daemon=True).start()
            self.thread = threading.Thread(target=self.run, daemon=True)
            self.thread.start()

    def stop(self):
        """Stops the vehicle and closes resources."""
        self.running = False
        if self.thread is not None:
            self.thread.join()
        self.send_sock.close()
        self.recv_sock.close()
        self.send_ack_sock.close()
        self.ack_sock.close()