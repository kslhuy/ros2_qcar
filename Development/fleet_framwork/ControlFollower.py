from ControlThread import ControlThread
from src.Controller.CACC import CACC
from src.Controller.idm_control import IDMControl
from QcarFleet import QcarFleet

from src.Controller.DummyController import DummyController, DummyVehicle
import numpy as np
import math, time, threading

from pal.utilities.math import wrap_to_pi

class ControlFollower(ControlThread):
    def __init__(self, SimulationTime:int, QcarFleet:QcarFleet, FollowerID:int, LeaderID:int, LookheadDistance:float = 7, MaxSteering:float = 0.6, Controller:str = "CACC", config=None):
        super().__init__()
        self.tf = SimulationTime
        self.Leader = QcarFleet.Qcars[LeaderID]
        self.Follower = QcarFleet.Qcars[FollowerID]

        # print("Follower:", FollowerID, "his Lead is:", LeaderID)

        self.FleetLock = QcarFleet.lock
        self.FollowerID = FollowerID
        self.LeaderID = LeaderID
        self.LookheadDistance = LookheadDistance

        self.prev_pos = None
        self.prev_time = None
        self.velocity = 0.0

        self.k_steering = 2.0 # Steering gain
        self.MaxSteering = MaxSteering

        # Use configuration if provided, otherwise use default parameters
        if config is not None:
            dummy_params = config.get_dummy_controller_params(FollowerID)
            DController = DummyController(FollowerID, dummy_params)
        else:
            DController = DummyController(FollowerID)
            
        match Controller:
            case "CACC":
                self.Controller = CACC(DController)
            case "IDM":
                self.Controller = IDMControl(DController)

        # Instantiate dummy_leader once for efficiency
        self.dummy_leader = DummyVehicle([0, 0, 0, 0], vehicle_id=self.LeaderID)
        pass

    def run(self):
        print(f"FollowerControl for Qcar Index {self.FollowerID} start, Leader Index: {self.LeaderID}")
        t0 = time.time()
        prev_time = t0
        twait = 3
        t = 0
        desired_update_rate = 100  # Hz (adjust as needed)
        desired_dt = 1.0 / desired_update_rate
        while t < self.tf + twait and not self._kill_thread.is_set():
            loop_start = time.time()
            with self.FleetLock:
                _, PosLeader, RotLeader, _     = self.Leader.get_world_transform()
                _, PosFollower, RotFollower, _ = self.Follower.get_world_transform()

            current_time = time.time()
            dt = current_time - prev_time  # Actual loop time
            prev_time = current_time       # Update previous time

            FollowerHeading = RotFollower[2]
            self._update_velocity(PosFollower)

            if self.LeaderID == 0:
                v_Leader = getattr(self.Leader, 'motorTach', 0.5)
            else:
                v_Leader = self.Leader.get_velocity()

            vFollower = self.get_velocity()
            FollowerState = [PosFollower[0], PosFollower[1], FollowerHeading, vFollower]
            LeaderState = [PosLeader[0], PosLeader[1], RotLeader[2], v_Leader]

            # Update dummy_leader's state instead of re-instantiating
            self.dummy_leader.state = LeaderState
            self.Controller.controller.get_surrounding_vehicles = lambda *args, **kwargs: (None, [self.dummy_leader], None, None)

            # Longitudinal Control
            _, ControlInput, _ = self.Controller.get_optimal_input(
                host_car_id=self.FollowerID,
                state=FollowerState,
                last_input=None,
                lane_id=None,
                input_log=None,
                initial_lane_id=None,
                direction_flag=None,
                type_state="true",
                acc_flag=0
            )
            SpeedCMD = ControlInput[0]

            # Lateral Control (pure pursuit)
            lookahead_distance = 0.5
            target_x = LeaderState[0] - lookahead_distance * math.cos(RotLeader[2])
            target_y = LeaderState[1] - lookahead_distance * math.sin(RotLeader[2])

            dx = target_x - FollowerState[0]
            dy = target_y - FollowerState[1]
            target_angle = math.atan2(dy, dx)
            heading_error = wrap_to_pi(target_angle - FollowerState[2])

            SteeringCMD = -self.k_steering * heading_error
            SteeringCMD = max(-self.MaxSteering, min(self.MaxSteering, SteeringCMD))

            self.APIControllerWrite(SpeedCMD, SteeringCMD)
            t = time.time() - t0

            # Print timing and control info for debugging
            print(f"Loop dt: {dt:.4f}s | SpeedCMD: {SpeedCMD:.3f} | SteeringCMD: {SteeringCMD:.3f}")

            # Enforce fixed update rate
            elapsed = time.time() - loop_start
            sleep_time = max(0, desired_dt - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)

        self.APIControllerWrite(0, 0)
        pass

    def _update_velocity(self, current_pos):
        current_time = time.time()
        if self.prev_pos is not None and self.prev_time is not None:
            dx = current_pos[0] - self.prev_pos[0]
            dy = current_pos[1] - self.prev_pos[1]
            distance = math.sqrt(dx**2 + dy**2)
            dt = current_time - self.prev_time
            self.velocity = distance / dt if dt > 1e-6 else 0.0
        else:
            self.velocity = 0.0
        self.prev_pos = current_pos
        self.prev_time = current_time

    def get_velocity(self):
        return self.velocity

    def APIControllerWrite(self, SpeedCMD, SteeringCMD):
        with self.FleetLock:
            self.Follower.set_velocity_and_request_state(
                forward         =SpeedCMD,
                turn            =SteeringCMD,
                headlights      =False,
                leftTurnSignal  =False,
                rightTurnSignal =False,
                brakeSignal     =False,
                reverseSignal   =False
            )
        pass

    def stop(self):
        super().stop()
