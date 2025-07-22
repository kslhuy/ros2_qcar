import socket
import json as ujson
import time
import logging
from md_logging_config import logger as parent_logger
from typing import Dict, Any
import threading

class CommHandler:
    """Handles communication for vehicle state, ACKs, and heartbeats."""
    def __init__(self, vehicle_id: int, target_ip: str, send_port: int, recv_port: int, ack_port: int, logger: logging.LoggerAdapter, running_flag):
        self.vehicle_id = vehicle_id
        self.target_ip = target_ip
        self.send_port = send_port
        self.recv_port = recv_port
        self.ack_port = ack_port
        self.logger = logger
        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock.bind(('0.0.0.0', recv_port))
        self.recv_sock.settimeout(0.01)
        self.send_ack_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ack_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ack_sock.settimeout(1.5)
        self.ack_sock.bind(('0.0.0.0', self.ack_port))
        self.logger.info(f"Bound ack_sock to port {self.ack_port}")
        self.sequence_number = 0
        self.last_heartbeat_time = time.time()
        self.heartbeat_timeout = 2.0
        self.heartbeat = True
        self.lock = threading.Lock()
        self.running = running_flag  # Reference to Vehicle's running flag



    # def send_state(self, current_pos: list, current_rot: list, velocity: float, synced_time: float):
    #     """Send vehicle state with ACK retry mechanism."""
    #     target_period = 0.1
    #     heartbeat_interval = 1.0
    #     max_retries = 3
    #     start_time = time.time()
    #     retries = 0
    #     ack_received = False

    #     while retries < max_retries and not ack_received:
    #     # while retries < max_retries:
    #         # print("pass1")
    #         print("ack recei0",ack_received)
    #         try:
    #             with self.lock:
    #                 # print("pass1.2")
    #                 data = {
    #                     'type': 'state',
    #                     'seq': self.sequence_number,
    #                     'id': self.vehicle_id,
    #                     'pos': current_pos,
    #                     'rot': current_rot,
    #                     'v': velocity,
    #                     'timestamp': synced_time,
    #                     'ack_port': self.ack_port
    #                 }
    #                 self.send_sock.sendto(ujson.dumps(data).encode(), (self.target_ip, self.send_port))
    #                 self.logger.info(f"SENT: Seq: {self.sequence_number}, Pos: {current_pos}, V: {velocity:.3f}")
    #                 # self.sequence_number += 1
    #                 try:
    #                     # print("try")
    #                     self.ack_sock.settimeout(1.0)
    #                     ack_data, addr = self.ack_sock.recvfrom(1024)  # Capture sender address
    #                     ack = ujson.loads(ack_data.decode())
    #                     print("ACK RECEIVED on", self.ack_port)

    #                     print(f"ACK CHECK: ack_id={ack.get('ack_id')}, self_id={self.vehicle_id}")

    #                     # print("ack",ack)
    #                     self.logger.debug(f"Received ACK from {addr}: {ack}, Expected seq: {self.sequence_number}")
    #                     if (ack.get('type') == 'ack' and 
    #                         ack.get('ack_seq') == self.sequence_number and 
    #                         ack.get('ack_id') != self.vehicle_id):
    #                     # if (ack.get('type') == 'ack'):
    #                         print("pass2")
    #                         ack_received = True
    #                         print("ack receiv 1",ack_received)
    #                         with self.lock:
    #                             self.sequence_number += 1
    #                             print("seq",self.sequence_number)
    #                         self.logger.info(f"ACK received for seq: {self.sequence_number - 1} from {addr}")
    #                         break
    #                     else:
    #                         self.logger.debug(f"ACK discarded: {ack}, Expected {self.sequence_number}")
    #                 except socket.timeout:
    #                     retries += 1
    #                     self.logger.warning(f"ACK timeout for seq: {self.sequence_number}, retry {retries}/{max_retries}")
    #                 except Exception as e:
    #                     self.logger.error(f"ACK ERROR: {e}, addr: {addr if 'addr' in locals() else 'N/A'}")
    #         except Exception as e:
    #             self.logger.error(f"SEND ERROR: {e}")
    #             retries += 1
    #             time.sleep(0.1)
    #             continue

    #     if not ack_received:
    #         self.logger.error(f"Failed to receive ACK for seq: {self.sequence_number} after {max_retries} retries")

    #     # Send heartbeat if interval elapsed
    #     if time.time() - self.last_heartbeat_time >= heartbeat_interval:
    #         try:
    #             heartbeat = {
    #                 'type': 'heartbeat',
    #                 'id': self.vehicle_id,
    #                 'timestamp': synced_time
    #             }
    #             self.send_sock.sendto(ujson.dumps(heartbeat).encode(), (self.target_ip, self.send_port))
    #             self.last_heartbeat_time = time.time()
    #             self.logger.info("SENT: Heartbeat")
    #         except Exception as e:
    #             self.logger.error(f"HEARTBEAT SEND ERROR: {e}")

    #     elapsed = time.time() - start_time
    #     sleep_time = max(0, target_period - elapsed)
    #     return sleep_time

    def send_state(self):
        """Continuously send vehicle state in a separate thread."""
        target_period = 0.1
        heartbeat_interval = 1.0
        while self.running:  # Use self.running from Vehicle class
            try:
                with self.lock:
                    current_pos = getattr(self, 'current_pos', [0, 0, 0])  # Fallback if not set
                    current_rot = getattr(self, 'current_rot', [0, 0, 0])
                    velocity = getattr(self, 'velocity', 0.0)
                    synced_time = getattr(self, 'synced_time', time.time())
                    data = {
                        'type': 'state',
                        'seq': self.sequence_number,
                        'id': self.vehicle_id,
                        'pos': current_pos,
                        'rot': current_rot,
                        'v': velocity,
                        'timestamp': synced_time,
                        'ack_port': self.ack_port
                    }
                    self.send_sock.sendto(ujson.dumps(data).encode(), (self.target_ip, self.send_port))
                    self.logger.info(f"SENT: Seq: {self.sequence_number}, Pos: {current_pos}, V: {velocity:.3f}")
                try:
                    self.ack_sock.settimeout(1.5)
                    ack_data, addr = self.ack_sock.recvfrom(1024)
                    ack = ujson.loads(ack_data.decode())
                    print("ACK RECEIVED on", self.ack_port)
                    print(f"ACK CHECK: ack_id={ack.get('ack_id')}, self_id={self.vehicle_id}")
                    self.logger.debug(f"Received ACK from {addr}: {ack}, Expected seq: {self.sequence_number}")
                    if (ack.get('type') == 'ack' and 
                        ack.get('ack_seq') == self.sequence_number and 
                        ack.get('ack_id') != self.vehicle_id):
                        print("pass2")
                        with self.lock:
                            self.sequence_number += 1
                        self.logger.info(f"ACK received for seq: {self.sequence_number - 1} from {addr}")
                    else:
                        self.logger.debug(f"ACK discarded: {ack}, Expected {self.sequence_number}")
                except socket.timeout:
                    self.logger.warning(f"ACK timeout for seq: {self.sequence_number}")
                except Exception as e:
                    self.logger.error(f"ACK ERROR: {e}, addr: {addr if 'addr' in locals() else 'N/A'}")
                if time.time() - self.last_heartbeat_time >= heartbeat_interval:
                    try:
                        heartbeat = {
                            'type': 'heartbeat',
                            'id': self.vehicle_id,
                            'timestamp': synced_time
                        }
                        self.send_sock.sendto(ujson.dumps(heartbeat).encode(), (self.target_ip, self.send_port))
                        self.last_heartbeat_time = time.time()
                        self.logger.info("SENT: Heartbeat")
                    except Exception as e:
                        self.logger.error(f"HEARTBEAT SEND ERROR: {e}")
                time.sleep(target_period)
            except Exception as e:
                self.logger.error(f"SEND STATE ERROR: {e}")
                time.sleep(0.1)

    # def receive_messages(self):
    #     """Continuously receive state, heartbeats, or ACKs in a separate thread."""
    #     target_period = 0.1
    #     while self.running:
    #         try:
    #             data, addr = self.recv_sock.recvfrom(1024)
    #             incoming = ujson.loads(data.decode())
    #             msg_type = incoming.get('type', '')
    #             self.logger.info(f"SENDING ACK to V{incoming['id']} from V{self.vehicle_id}")
    #             if msg_type == 'state' and incoming.get('id') != self.vehicle_id:
    #                 seq = incoming.get('seq', -1)
    #                 self.logger.info(f"RECEIVED: Seq: {seq}, Sender ID: {incoming['id']}, Pos: {incoming['pos']}")
    #                 try:
    #                     ack = {'type': 'ack', 'ack_seq': seq, 'ack_id': self.vehicle_id}
    #                     sender_ack_port = incoming.get('ack_port')
    #                     self.send_ack_sock.sendto(ujson.dumps(ack).encode(), (addr[0], sender_ack_port))
    #                     self.logger.info(f"SENT: ACK for seq: {seq} to port {sender_ack_port}, from {addr}, time: {time.time()}")
    #                     # Store the received state as leader_state if it's from the leader
    #                     if incoming.get('id') == 0:  # Assuming vehicle_id 0 is the leader
    #                         with self.lock:
    #                             self.leader_state = incoming
    #                 except Exception as e:
    #                     self.logger.error(f"ACK SEND ERROR: {e}, addr: {addr}, time: {time.time()}")
    #             elif msg_type == 'heartbeat':
    #                 self.last_heartbeat_time = time.time()
    #                 self.heartbeat = True
    #                 self.logger.info(f"RECEIVED: Heartbeat from V{incoming['id']}")
    #             elif msg_type == 'ack':
    #                 pass
    #             else:
    #                 self.logger.warning(f"Unknown message type received: {msg_type}")
    #         except socket.timeout:
    #             if time.time() - self.last_heartbeat_time > self.heartbeat_timeout:
    #                 self.heartbeat = False
    #                 self.logger.error("No heartbeat received for over 2 seconds, assuming leader failure")
    #         except Exception as e:
    #             self.logger.error(f"RECEIVE ERROR: {e}")
    #         time.sleep(target_period)

    def receive_messages(self):
        """Receive state, heartbeats, or ACKs."""
        target_period = 0.1
        start_time = time.time()
        result = None
        try:
            while time.time() - start_time < 0.09:  # Allow 90ms for multiple packets
                # print("rec1")
                try:
                    data, addr = self.recv_sock.recvfrom(1024)
                    incoming = ujson.loads(data.decode())
                    msg_type = incoming.get('type', '')
                    # print("msg type",msg_type)
                    self.logger.info(f"SENDING ACK to V{incoming['id']} from V{self.vehicle_id}")

                    if msg_type == 'state' and incoming.get('id') != self.vehicle_id:
                    # if msg_type == 'state' :    
                        # print("rec2")
                        seq = incoming.get('seq', -1)
                        self.logger.info(f"RECEIVED: Seq: {seq}, Sender ID: {incoming['id']}, Pos: {incoming['pos']}")
                        try:
                            ack = {'type': 'ack', 'ack_seq': seq, 'ack_id': self.vehicle_id}
                            sender_ack_port = incoming.get('ack_port')
                            self.send_ack_sock.sendto(ujson.dumps(ack).encode(), (addr[0], sender_ack_port))
                            self.logger.info(f"SENT: ACK for seq: {seq} to port {sender_ack_port}")
                        except Exception as e:
                            self.logger.error(f"ACK SEND ERROR: {e}")
                        result = incoming  # Update result with the latest state
                        # print("result",result)
                    elif msg_type == 'heartbeat':
                        self.last_heartbeat_time = time.time()
                        self.heartbeat = True
                        self.logger.info(f"RECEIVED: Heartbeat from V{incoming['id']}")
                    elif msg_type == 'ack':
                        pass
                    else:
                        self.logger.warning(f"Unknown message type received: {msg_type}")
                except socket.timeout:
                    break  # Exit inner loop on timeout

        except socket.timeout:
            if time.time() - self.last_heartbeat_time > self.heartbeat_timeout:
                self.heartbeat = False
                self.logger.error("No heartbeat received for over 2 seconds, assuming leader failure")
        except Exception as e:
            elapsed = time.time() - start_time
            self.logger.error(f"RECEIVE ERROR: {e}, Elapsed: {elapsed:.6f} s")

        elapsed = time.time() - start_time
        sleep_time = max(0, target_period - elapsed)
        return result, sleep_time  # Returns a tuple of (result, sleep_time)

    def cleanup(self):
        """Close all sockets."""
        self.send_sock.close()
        self.recv_sock.close()
        self.send_ack_sock.close()
        self.ack_sock.close()