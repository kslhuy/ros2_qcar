import math
import numpy as np
import threading
import time
from Controller.idm_control import IDMControl
import os


class Follower:
    def __init__(self, qcar, idm_controller, vehicle_id=1, max_steering=0.6):
        self.qcar = qcar
        self.idm = idm_controller
        self.max_steering = max_steering
        self.id = vehicle_id 

        self.running = False
        self.thread = None

        self.history = {
            "time": [],
            "pos_leader": [],
            "pos_follower": [],
            "vel_leader": [],
            "vel_follower": [],
            "acc_leader": [],
            "acc_follower": [],
            "spacing_theory": []

        }

        # Velocity calculation
        self.prev_pos = None
        self.prev_time = None
        self.velocity = 0.5  # Initial velocity estimate

        
    def wrap_to_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def follow(self, leader):
        # Get positions and orientations
        _, pos_leader, rot_leader, _ = leader.get_world_transform()    
        _, pos_follower, rot_follower, _ = self.qcar.get_world_transform()

        # Calculate velocity using position difference
        current_time = time.time()
        if self.prev_pos is not None and self.prev_time is not None:
            # Compute Euclidean distance between current and previous position
            dx = pos_follower[0] - self.prev_pos[0]
            dy = pos_follower[1] - self.prev_pos[1]
            distance = math.sqrt(dx**2 + dy**2)
            dt = current_time - self.prev_time
            if dt > 0:  # Avoid division by zero
                self.velocity = distance / dt
            else:
                self.velocity = 0.0
        self.prev_pos = pos_follower
        self.prev_time = current_time

        lookahead_distance = 0.4  # meters
        target_x = pos_leader[0] - lookahead_distance * math.cos(rot_leader[2])
        target_y = pos_leader[1] - lookahead_distance * math.sin(rot_leader[2])

        dx = target_x - pos_follower[0]
        dy = target_y - pos_follower[1]

        # dx_truth = pos_leader[0] - pos_follower[0]
        # dy_truth = pos_leader[1] - pos_follower[1]
        # distance = math.hypot(dx_truth, dy_truth)
        # print('distance m',distance)

        follower_heading = rot_follower[2]
        target_angle = math.atan2(dy, dx)
        heading_error = self.wrap_to_pi(target_angle - follower_heading)

        # Steering using simple P controller
        steering_cmd = -2.0 * heading_error  # could be a param
        steering_cmd = max(-self.max_steering, min(self.max_steering, steering_cmd))
        # print('steering_cmd ok',steering_cmd)

        # Form follower state: [x, y, theta, v]
        # Approximate velocity using past positions or from QCar if available
        v_follower = self.velocity
        print('v_follo',v_follower)        
        follower_state = [pos_follower[0], pos_follower[1], follower_heading, v_follower]
        # print('follower state',follower_state)


        class DummyVehicle:
            def __init__(self, state, vehicle_number=0):
                self.state = state
                self.vehicle_number = vehicle_number


        leader_state = [pos_leader[0], pos_leader[1], rot_leader[2], v_follower]
        print('leader',leader_state)
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



        speed_cmd = max(0, input_u[0])  # throttle only, no braking for now
        # print('speed cmd',speed_cmd)
        

        # print("Let's go")

        # Send to QCar
        self.qcar.set_velocity_and_request_state(
            forward=speed_cmd,
            turn=steering_cmd,
            headlights=False,
            leftTurnSignal=False,
            rightTurnSignal=False,
            brakeSignal=False,
            reverseSignal=False
        )

        now = time.time()

        s0 = self.idm.param_opt['ri']
        h = self.idm.param_opt['hi']
        v_f = v_follower  # already computed earlier

        spacing_theory = s0 + h * v_f


        # Approximate velocity (forward component only)
        if len(self.history["time"]) > 1:
            dt = now - self.history["time"][-1]
            prev_p_l = self.history["pos_leader"][-1]
            prev_p_f = self.history["pos_follower"][-1]

            v_l = [(pos_leader[0]-prev_p_l[0])/dt, (pos_leader[1]-prev_p_l[1])/dt]
            v_f = [(pos_follower[0]-prev_p_f[0])/dt, (pos_follower[1]-prev_p_f[1])/dt]

            prev_v_l = self.history["vel_leader"][-1]
            prev_v_f = self.history["vel_follower"][-1]

            a_l = [(v_l[0]-prev_v_l[0])/dt, (v_l[1]-prev_v_l[1])/dt]
            a_f = [(v_f[0]-prev_v_f[0])/dt, (v_f[1]-prev_v_f[1])/dt]
        else:
            v_l = v_f = a_l = a_f = [0.0, 0.0]

        # Append to history
        self.history["time"].append(now)
        self.history["pos_leader"].append(pos_leader)
        self.history["pos_follower"].append(pos_follower)
        self.history["vel_leader"].append(v_l)
        self.history["vel_follower"].append(v_f)
        self.history["acc_leader"].append(a_l)
        self.history["acc_follower"].append(a_f)
        self.history["spacing_theory"].append(spacing_theory)



    def save_history(self, path="relative_states.npz"):
        t = np.array(self.history["time"]) - self.history["time"][0]
        pos_l = np.array(self.history["pos_leader"])
        pos_f = np.array(self.history["pos_follower"])
        vel_l = np.array(self.history["vel_leader"])
        vel_f = np.array(self.history["vel_follower"])
        acc_l = np.array(self.history["acc_leader"])
        acc_f = np.array(self.history["acc_follower"])
        spacing_th = np.array(self.history["spacing_theory"])


        np.savez(path,
            time=t,
            pos_leader=pos_l,
            pos_follower=pos_f,
            vel_leader=vel_l,
            vel_follower=vel_f,
            acc_leader=acc_l,
            acc_follower=acc_f,
            spacing_theory=spacing_th
        )
        print(f"[INFO] Saved relative state log to {path}")


        
    def run(self, leader):
        """Start following leader in a loop."""
        self.running = True
        while self.running:
            self.follow(leader)
            time.sleep(0.2)  # Small delay to simulate a realistic loop, adjust as needed
        # self.save_history(path="relative_states.npz")


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