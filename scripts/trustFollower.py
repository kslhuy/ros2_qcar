import math
import numpy as np
import threading
import time
from Controller.idm_control import IDMControl

from Trust.TriPTrustModel import TriPTrustModel
from UDPListener.LeaderUDPListener import LeaderUDPListener
from Controller.ControllerFollower import ControllerFollower

import socket
import json
import threading
import pickle

class Follower:
    def __init__(self, qcar, idm_controller, vehicle_id,
                 k_steering=2.0, max_steering=0.6,
                 max_speed=1.0, safe_distance=0.5):
        self.qcar = qcar
        self.k_steering = k_steering
        self.max_steering = max_steering
        self.max_speed = max_speed
        self.default_safe_distance = safe_distance
        self.idm = idm_controller
        self.id = vehicle_id

        self.trust_model = TriPTrustModel()
        self.udp_listener = LeaderUDPListener(port=9999, use_pickle=False)
        self.udp_listener.start()

        self.control = ControllerFollower(self.qcar, self.trust_model)
        self.running = False
        self.thread = None

        self.trust_log = []


    def wrap_to_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi


    def run(self):
        self.running = True
        while self.running:
            msg = self.udp_listener.get_latest()
            # print('msg before',msg)
            if msg:
                leader_state = {
                    'position': [msg.get('x', 0.0), msg.get('y', 0.0), 0.005],
                    'rotation': [0, 0, msg.get('theta', 0.0)],
                    'velocity': msg.get('v', 0.0),
                    'acceleration': msg.get('a', 0.0)
                }

                self.control.update_leader_state(leader_state)
                print("[Follower] Received:",leader_state)
                # Log trust score and decision factors
                trust_score = self.trust_model.calculate_trust_score(self.trust_model.rating_vector)
                gamma = self.trust_model.gamma_cross_log[-1] if self.trust_model.gamma_cross_log else 0
                d_score = self.trust_model.d_score_log[-1] if self.trust_model.d_score_log else 0
                v_score = self.trust_model.v_score_log[-1] if self.trust_model.v_score_log else 0

                self.trust_log.append([time.time(), trust_score, gamma, v_score, d_score])

            else:
                print("[Follower] Waiting...")

            self.control.step()
            time.sleep(0.5)

    # def run(self):
    #     self.running = True
    #     while self.running:
    #         msg = self.udp_listener.get_latest()
    #         if msg:
    #             print("[Follower] Received:", msg)
    #         else:
    #             print("[Follower] Waiting...")
    #         time.sleep(0.5)



    def start(self, leader):
        """Start the follower loop in a new thread."""
        if self.thread is None or not self.thread.is_alive():
            self.thread = threading.Thread(target=self.run, args=(leader,))
            self.thread.start()

    def stop(self):
        """Stop the follower loop."""
        self.running = False
        if self.thread is not None:
            self.thread.join()