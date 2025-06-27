import math
import numpy as np
import threading
import time
from Controller.idm_control import IDMControl
from Trust.TriPTrustModel2 import TriPTrustModel

class Follower:
    def __init__(self, qcar, idm_controller, vehicle_id=1, max_steering=0.6):
        self.qcar = qcar
        self.idm = idm_controller
        self.max_steering = max_steering
        self.id = vehicle_id 
        self.running = False
        self.thread = None
        self.trust_model = TriPTrustModel()

    def wrap_to_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def follow(self, leader):
        try:
            _, pos_leader, rot_leader, _ = leader.get_world_transform()
        except Exception as e:
            print(f"[Follower-{self.id}] get_world_transform() failed: {e}")
            return
        _, pos_follower, rot_follower, _ = self.qcar.get_world_transform()

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

        leader_state = [pos_leader[0], pos_leader[1], rot_leader[2], v_follower]

        class DummyVehicle:
            def __init__(self, state, vehicle_number=0):
                self.state = state
                self.vehicle_number = vehicle_number

        dummy_leader = DummyVehicle(leader_state, vehicle_number=self.id - 1)
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

        trust_score = self.trust_model.determine_following_distance(
            trust_score=0.7,  # Example, you could compute actual trust here if wrapped in vehicle classes
            ds=0.5,
            dacc=1.5
        )

        speed_cmd = max(0, input_u[0])
        distance = math.hypot(dx, dy)
        if distance < trust_score:
            speed_cmd = 0

        self.qcar.set_velocity_and_request_state(
            forward=speed_cmd,
            turn=steering_cmd,
            headlights=False,
            leftTurnSignal=False,
            rightTurnSignal=False,
            brakeSignal=False,
            reverseSignal=False
        )

    def run(self, leader):
        self.running = True
        while self.running:
            self.follow(leader)
            time.sleep(0.2)

    def start(self, leader):
        if self.thread is None or not self.thread.is_alive():
            self.thread = threading.Thread(target=self.run, args=(leader,))
            self.thread.start()

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join()
