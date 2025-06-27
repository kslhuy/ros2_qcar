import math
import numpy as np
import threading
import time
from Controller.idm_control import IDMControl


class Follower:
    def __init__(self, qcar, idm_controller, vehicle_id=1, max_steering=0.6):
        self.qcar = qcar
        self.idm = idm_controller
        self.max_steering = max_steering
        self.id = vehicle_id 

        self.running = False
        self.thread = None

        self.observer = None  # Optional observer object

        self.true_state_log = []
        self.time_log = []



    def set_observer(self, observer):
        self.observer = observer


    def wrap_to_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def follow(self, leader):
        # Get positions and orientations
        if self.observer is not None:
            est_state = self.observer.est_global_state_current[:, 0]  # assuming 1 leader (index 0)
            pos_leader = est_state[:2]
            theta_leader = est_state[2]
            print('theta',theta_leader)
            print('pos',pos_leader)
        else:
            _, pos_leader, rot_leader, _ = leader.get_world_transform()
            theta_leader = rot_leader[2]
    
        _, pos_follower, rot_follower, _ = self.qcar.get_world_transform()

        lookahead_distance = 0.4  # meters
        target_x = pos_leader[0] - lookahead_distance * math.cos(theta_leader)
        target_y = pos_leader[1] - lookahead_distance * math.sin(theta_leader)

        dx = target_x - pos_follower[0]
        dy = target_y - pos_follower[1]

        # dx = pos_leader[0] - pos_follower[0]
        # dy = pos_leader[1] - pos_follower[1]
        # distance = math.hypot(dx, dy)
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
        v_follower = self.qcar.motorTach if hasattr(self.qcar, 'motorTach') else 0.5
        follower_state = [pos_follower[0], pos_follower[1], follower_heading, v_follower]



        class DummyVehicle:
            def __init__(self, state, vehicle_number=0):
                self.state = state
                self.vehicle_number = vehicle_number


        leader_state = [pos_leader[0], pos_leader[1], theta_leader, v_follower]
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

        
    def run(self, leader):
        self.running = True
        instant_index = 0

        while self.running:
            # Update observer if available
            if self.observer:
                _, pos, rot, _ = self.qcar.get_world_transform()

                true_state = np.array([pos[0], pos[1], rot[2], 0.5])
                self.true_state_log.append(true_state)
                self.time_log.append(time.time())

                v = 0.5  # You can later compute this from odometry if available
                measured_state = np.array([pos[0], pos[1], rot[2], v])

                self.observer.local_observer(measured_state, instant_index)
                self.observer.distributed_observer(instant_index, weights=np.ones(3))  # example weights
                print("Updated est_global_state_current:", self.observer.est_global_state_current[:, 0])

                instant_index += 1

            self.follow(leader)
            time.sleep(0.1)


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