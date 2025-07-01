import math
import numpy as np
import threading
import time
from Controller.idm_control import IDMControl

import socket
import pickle

class Follower:
    def __init__(self, qcar, idm_controller, vehicle_id=1, max_steering=0.6):
        self.qcar = qcar
        self.idm = idm_controller
        self.max_steering = max_steering
        self.id = vehicle_id 

        self.running = False
        self.thread = None

        self.follower_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.leader_ip = '127.0.0.1'
        self.leader_port = 5050

        self.sender_thread = threading.Thread(target=self.send_to_leader, daemon=True)
        # self.sender_thread.start()

        def delayed_start():
            time.sleep(1.5)  # give QLabs time to be ready
            self.sender_thread.start()

        threading.Thread(target=delayed_start, daemon=True).start()

        

        self.leader_state = {'pos': [0, 0, 0], 'rot': [0, 0, 0], 'v': 0.3}
        self.last_seq = -1

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 5005))

        self.network_thread = threading.Thread(target=self.listen_for_leader, daemon=True)
        self.network_thread.start()

    def send_to_leader(self):
        while True:
            try:
                _, pos, rot, _ = self.qcar.get_world_transform()
                # v = self.qcar.motorTach
                v = 0.5
                data = {'pos': pos, 'rot': rot, 'v': v}
                self.follower_sock.sendto(pickle.dumps(data), (self.leader_ip, self.leader_port))
                print("sent to leader")
            except Exception as e:
                print("Follower send error:", e)
                time.sleep(1.0)  # wait longer after a failure
                continue
            time.sleep(0.1)

    # def send_to_leader(self):
    #     counter = 0
    #     while True:
    #         print(f"Test message {counter}")
    #         counter += 1
    #         time.sleep(1)


    # def listen_for_leader(self):
    #     while True:
    #         data, _ = self.sock.recvfrom(1024)
    #         self.leader_state = pickle.loads(data)
    #         # print("received from leader")
            
    def listen_for_leader(self):
        while True:
            data, _ = self.sock.recvfrom(1024)
            incoming = pickle.loads(data)

            seq = incoming.get('seq', -1)

            # Always print, even if no packets are missed
            if self.last_seq != -1:
                missed = seq - self.last_seq - 1
                print(f"Seq received: {seq} | Last: {self.last_seq} | Missed: {missed}")
            else:
                print(f"Seq received: {seq} (initial packet)")

            self.last_seq = seq
            self.leader_state = incoming

    def wrap_to_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def follow(self):

        try:
            _, pos_follower, rot_follower, _ = self.qcar.get_world_transform()
        except Exception as e:
            print("Follower read error:", e)
            return  # Skip this cycle safely
        
        pos_leader = self.leader_state['pos']
        rot_leader = self.leader_state['rot']
        v_leader = self.leader_state['v']

        # _, pos_follower, rot_follower, _ = self.qcar.get_world_transform()

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

        v_follower = self.qcar.motorTach if hasattr(self.qcar, 'motorTach') else 0.5
        follower_state = [pos_follower[0], pos_follower[1], follower_heading, v_follower]

        class DummyVehicle:
            def __init__(self, state, vehicle_number=0):
                self.state = state
                self.vehicle_number = vehicle_number

        leader_state = [pos_leader[0], pos_leader[1], rot_leader[2], v_leader]
        dummy_leader = DummyVehicle(leader_state, vehicle_number=0)
        self.idm.controller.get_surrounding_vehicles = lambda *args, **kwargs: (None, [dummy_leader], None, None)

        _, input_u, _ = self.idm.get_optimal_input(
            host_car_id=self.id,
            state=follower_state,
            last_input=None,
            lane_id=None,
            input_log=None,
            initial_lane_id=None,
            direction_flag=None,
            type_state="true",
            acc_flag=0
        )

        speed_cmd = max(0, input_u[0])

        self.qcar.set_velocity_and_request_state(
            forward=speed_cmd,
            turn=steering_cmd,
            headlights=False,
            leftTurnSignal=False,
            rightTurnSignal=False,
            brakeSignal=False,
            reverseSignal=False
        )

    def run(self):
        self.running = True
        while self.running:
            self.follow()
            time.sleep(0.2)

    # def start(self):
    #     if self.thread is None or not self.thread.is_alive():
    #         self.thread = threading.Thread(target=self.run)
    #         self.thread.start()

    # def start(self, leader):
    #     """Start the follower loop in a new thread."""
    #     if self.thread is None or not self.thread.is_alive():
    #         self.thread = threading.Thread(target=self.run, args=(leader,))
    #         self.thread.start()

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join()
