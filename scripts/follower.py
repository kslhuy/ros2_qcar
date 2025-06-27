import math
import numpy as np
import threading
import time

class Follower:
    def __init__(self, qcar, k_steering=3.0, max_steering=0.6, max_speed=1.0, safe_distance=0.42):
        self.qcar = qcar
        self.k_steering = k_steering
        self.max_steering = max_steering
        self.max_speed = max_speed
        self.safe_distance = safe_distance
        
        self.running = False
        self.thread = None
        
    def wrap_to_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def follow(self, leader):
        _, pos_leader, rot_leader, _ = leader.get_world_transform()    
        _, pos_follower, rot_follower, _ = self.qcar.get_world_transform()
        
        
        # Vector from follower to leader
        dx = pos_leader[0] - pos_follower[0]
        dy = pos_leader[1] - pos_follower[1]
        distance = math.hypot(dx, dy)

        # print('pos',pos_leader)

        # Follower heading and target direction
        follower_heading = rot_follower[2]
        target_angle = math.atan2(dy, dx)
        heading_error = target_angle - follower_heading
        heading_error = self.wrap_to_pi(heading_error)

        # Steering command
        steering_cmd = -self.k_steering * heading_error
        steering_cmd = max(-self.max_steering, min(self.max_steering, steering_cmd))

        # Speed control: Stop if too close
        if distance < self.safe_distance:
            speed_cmd = 0.0
        else:
            speed_cmd = min(self.max_speed, 2.2 * (distance - self.safe_distance))
            # print("speed",speed_cmd)

        # Move follower
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
        """Stop the follower loop."""
        self.running = False
        if self.thread is not None:
            self.thread.join()