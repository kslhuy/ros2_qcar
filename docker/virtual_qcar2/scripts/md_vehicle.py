from md_gps_sync import GPSSync
from md_comm_handler import CommHandler
import logging
from md_logging_config import logger as parent_logger
from typing import Any
import time
import math
import threading

class Vehicle:
    """Represents a vehicle in a platoon, supporting leader or follower roles with QLabs integration."""
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
        self.logger = logging.LoggerAdapter(parent_logger, {'vehicle_id': vehicle_id})
        self.comm = CommHandler(vehicle_id, ip, send_port, recv_port, ack_port, self.logger, self.running)

        self.leader_state = {'pos': [-1.205, -0.83, 0.005], 'rot': [0, 0, -44.7], 'v': 0.3}
        self.last_seq = -1
        self.last_sync_attempt = time.time()
        self.sync_interval = 5.0

        self.current_pos = [0, 0, 0]
        self.current_rot = [0, 0, 0]
        self.velocity = 0.0
        self.prev_pos = None
        self.prev_time = None
    # def __init__(self, vehicle_id, is_leader, qcar,idm_controller=None, 
    #              target_ip="127.0.0.1", send_port=5000, recv_port=5001, 
    #              ack_port=6000):
    #     self.vehicle_id = vehicle_id
    #     self.is_leader = is_leader
    #     self.qcar = qcar
    #     self.gps_sync = GPSSync()
    #     self.target_ip = target_ip
    #     self.send_port = send_port
    #     self.recv_port = recv_port
    #     self.ack_port = ack_port
    #     self.logger = logging.LoggerAdapter(parent_logger, {'vehicle_id': vehicle_id})
    #     self.running = False
    #     self.comm = CommHandler(vehicle_id, target_ip, send_port, recv_port, ack_port, self.logger, self.running)
    #     self.thread = None
    #     self.current_pos = [0, 0, 0]
    #     self.current_rot = [0, 0, 0]
    #     self.velocity = 0.0
    #     self.leader_state = None
    #     self.lock = threading.Lock()
    #     self.last_sync_attempt = 0
    #     self.sync_interval = 1.0
    #     self.idm = idm_controller
    #     self.prev_pos = None
    #     self.prev_time = None

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
            if not self.comm.heartbeat:
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

            self.current_pos = pos_follower
            self.current_rot = rot_follower

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

            v_follower = self.velocity
            follower_state = [pos_follower[0], pos_follower[1], follower_heading, v_follower]
            class DummyVehicle:
                def __init__(self, state, vehicle_number=0):
                    self.state = state
                    self.vehicle_number = vehicle_number
            leader_state = [pos_leader[0], pos_leader[1], rot_leader[2], v_leader]
            # print("leader state",leader_state)
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

    # def run(self):
    #     """Runs the vehicle control loop at 10 Hz."""
    #     self.logger.info(f"Starting run loop, is_leader: {self.is_leader}")
    #     self.running = True
    #     self.gps_sync.sync_with_gps()
    #     while self.running:
    #         # print("running")
    #         start_time = time.time()
    #         if start_time - self.last_sync_attempt >= self.sync_interval:
    #             self.gps_sync.sync_with_gps()
    #             self.last_sync_attempt = start_time

    #         self.update_movement()
    #         sleep_time = self.comm.send_state(self.current_pos, self.current_rot, self.velocity, self.gps_sync.get_synced_time())
    #         # print(self.comm.receive_messages())
    #         received_state, recv_sleep = self.comm.receive_messages()
    #         # print(received_state)
    #         if received_state and not self.is_leader:
    #             with self.lock:
    #                 self.leader_state = received_state
    #                 self.logger.info(f"Updated leader_state: {self.leader_state}")
    # #         time.sleep(max(sleep_time, recv_sleep))

    # def run(self):
    #     """Runs the vehicle control loop at 10 Hz."""
    #     self.logger.info(f"Starting run loop, is_leader: {self.is_leader}")
    #     while self.running:
    #         start_time = time.time()
    #         self.logger.debug(f"Run loop iteration, running: {self.running}")
    #         try:
    #             if start_time - self.last_sync_attempt >= self.sync_interval:
    #                 self.gps_sync.sync_with_gps()
    #                 self.last_sync_attempt = start_time

    #             self.update_movement()
    #             sleep_time = self.comm.send_state(self.current_pos, self.current_rot, self.velocity, self.gps_sync.get_synced_time())
    #             received_state, recv_sleep = self.comm.receive_messages()
    #             self.logger.debug(f"Received state: {received_state}")
    #             if received_state and not self.is_leader:
    #                 with self.lock:
    #                     self.leader_state = received_state
    #                     self.logger.info(f"Updated leader_state: {self.leader_state}")
    #             time.sleep(max(sleep_time, recv_sleep))
    #         except Exception as e:
    #             self.logger.error(f"Run loop error: {e}, continuing")

    def run(self):
        """Runs the vehicle control loop at 10 Hz for coordination."""
        self.logger.info(f"Starting run loop, is_leader: {self.is_leader}")
        while self.running:
            start_time = time.time()
            try:
                if start_time - self.last_sync_attempt >= self.sync_interval:
                    self.gps_sync.sync_with_gps()
                    self.last_sync_attempt = start_time
                self.update_movement()
                # State and reception handled by separate threads
                if not self.is_leader and hasattr(self.comm, 'leader_state'):
                    with self.lock:
                        self.leader_state = getattr(self.comm, 'leader_state', None)
                        if self.leader_state:
                            self.logger.info(f"Updated leader_state: {self.leader_state}")
                time.sleep(max(0, 0.1 - (time.time() - start_time)))  # Maintain 10 Hz
            except Exception as e:
                self.logger.error(f"Run loop error: {e}, continuing")

    # def start(self):
    #     """Starts the vehicle threads."""
    #     self.logger.info(f"Starting vehicle, is_leader: {self.is_leader}")
    #     if self.thread is None or not self.thread.is_alive():
    #         # print("start")
    #         time.sleep(0.5)
    #         self.running = True
    #         self.thread = threading.Thread(target=self.run, daemon=True)
    #         self.thread.start()
    #         self.logger.debug(f"Started thread {self.thread.name}, is_alive: {self.thread.is_alive()}")\

    def start(self):
        """Starts the vehicle threads."""
        self.logger.info(f"Starting vehicle, is_leader: {self.is_leader}")
        if self.thread is None or not self.thread.is_alive():
            self.running = True
            threading.Thread(target=self.comm.send_state, daemon=True).start()  # Use CommHandler method
            threading.Thread(target=self.comm.receive_messages, daemon=True).start()  # Use CommHandler method
            self.thread = threading.Thread(target=self.run, daemon=True)
            self.thread.start()
            self.logger.debug(f"Started threads: send_state, receive_messages, run, thread alive: {self.thread.is_alive()}")

    def stop(self):
        """Stops the vehicle and closes resources."""
        self.running = False
        if self.thread is not None:
            self.thread.join()
        self.comm.cleanup()