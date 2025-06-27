import math
import numpy as np
import threading
import time

from Controller.idm_control import IDMControl
from Controller.CACC import CACC
# from Controller.look_ahead_Control import LookAheadControl

class Follower:
    def __init__(self, qcar, controller, vehicle_id=1, max_steering=0.6):
        self.qcar = qcar
        self.controller = controller  # Can be IDM, CACC, or LookAhead
        self.max_steering = max_steering
        self.id = vehicle_id

        self.start_time = None

        self.log_file = open(f"follower_log_{self.id}.csv", "w")
        self.log_file.write("time,speed_cmd,steering_cmd\n")  # CSV header




        
    def wrap_to_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def follow(self, leader):
        elapsed = time.time() - self.start_time

        # print(f"Using controller: {self.controller.__class__.__name__}")
        # Get positions and orientations
        _, pos_leader, rot_leader, _ = leader.get_world_transform()    
        _, pos_follower, rot_follower, _ = self.qcar.get_world_transform()

        dx = pos_leader[0] - pos_follower[0]
        dy = pos_leader[1] - pos_follower[1]
        distance = math.hypot(dx, dy)
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


        leader_state = [pos_leader[0], pos_leader[1], rot_leader[2], v_follower]
        dummy_leader = DummyVehicle(leader_state, vehicle_number=0)
        self.controller.controller.get_surrounding_vehicles = lambda *args, **kwargs: (None, [dummy_leader], None, None)


        _, input_u, _ = self.controller.get_optimal_input(
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

        # if isinstance(self.controller, (IDMControl,CACC)):  # optional
        #     heading_error = self.wrap_to_pi(target_angle - follower_heading)
        #     steering_cmd = -2.0 * heading_error
        # else:
        #     steering_cmd = input_u[1]

        # print('timer (s)',elapsed)

        # if elapsed < 16:
        #     # Use simple heading controller for first 10 seconds
        #     heading_error = self.wrap_to_pi(target_angle - follower_heading)
        #     steering_cmd = -2.0 * heading_error
        #     print("Using P-steering (init phase)")
        # else:
        #     # Use controller's steering logic
        #     steering_cmd = input_u[1]
        #     print("Using controller steering")

            
        steering_cmd = max(-self.max_steering, min(self.max_steering, steering_cmd))  # clip if needed
        # print('steering_cmd',steering_cmd)


        speed_cmd = max(0.3, input_u[0])  # throttle only, no braking for now
        # print('speed_cmd',speed_cmd)

        timestamp = time.time() - self.start_time
        self.log_file.write(f"{timestamp:.2f},{speed_cmd:.4f},{steering_cmd:.4f}\n")

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
        self.start_time = time.time()

        """Start following leader in a loop."""
        self.running = True
        while self.running:
            self.follow(leader)
            time.sleep(0.2)  # Small delay to simulate a realistic loop, adjust as needed

    def start(self, leader):
        """Start the follower loop in a new thread."""
        if self.thread is None or not self.thread.is_alive():
            self.thread = threading.Thread(target=self.run, args=(leader,))
            self.thread.start()

    def stop(self):
        if self.log_file:
            self.log_file.close()

        """Stop the follower loop."""
        self.running = False
        if self.thread is not None:
            self.thread.join()