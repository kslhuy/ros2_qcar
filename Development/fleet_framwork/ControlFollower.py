from ControlThread import ControlThread
from src.Controller.CACC import CACC
from src.Controller.idm_control import IDMControl
from QcarFleet import QcarFleet

from src.Controller.DummyController import DummyController, DummyVehicle
import numpy as np
import math, time, threading

from pal.utilities.math import wrap_to_pi

class ControlFollower(ControlThread):
    def __init__(self, SimulationTime:int, QcarFleet:QcarFleet, FollowerID:int, LeaderID:int, LookheadDistance:float = 7, MaxSteering:float = 0.6, Controller:str = "CACC"):
        super().__init__()
        self.tf = SimulationTime
        self.Leader = QcarFleet.Qcars[LeaderID]
        self.Follower = QcarFleet.Qcars[FollowerID]
        self.FleetLock = QcarFleet.lock
        self.FollowerID = FollowerID
        self.LeaderID = LeaderID
        self.LookheadDistance = LookheadDistance
        self.MaxSteering = MaxSteering

        DController = DummyController(FollowerID)
        match Controller:
            case "CACC":
                self.Controller = CACC(DController)
            case "IDM":
                self.Controller = IDMControl(DController)
        pass

    def run(self):
        print("FollowerControl for Qcar Index ",self.FollowerID," start, Leader Index:", self.LeaderID)
        t0 = time.time()
        twait = 3
        t = 0
        while t < self.tf + twait:
            with self.FleetLock:
                _, PosLeader, RotLeader, _     = self.Leader.get_world_transform()
                _, PosFollower, RotFollower, _ = self.Follower.get_world_transform()

            #region: LookheadDistance not function, need to be revised
            TargetX = PosLeader[0] - self.LookheadDistance * math.cos(RotLeader[2])
            TargetY = PosLeader[1] - self.LookheadDistance * math.sin(RotLeader[2])
            #endregion

            # dx = TargetX - PosFollower[0]
            # dy = TargetY - PosLeader[1]
            FollowerHeading = RotFollower[2]
            # TargetAngle = math.atan2(dy, dx)
            # HeadingError = wrap_to_pi(TargetAngle - FollowerHeading)

            # SteeringCMD = max(-self.MaxSteering, min(self.MaxSteering, 5.0 * HeadingError))
            SteeringCMD = 0

            vFollower = getattr(self.Follower, 'motorTach', 0.5)
            FollowerState = [PosFollower[0], PosFollower[1], FollowerHeading, vFollower]
            LeaderState = [TargetX, TargetY, RotLeader[2], vFollower]

            #region:DummyController Part
            dummy_leader = DummyVehicle(LeaderState, vehicle_id = self.LeaderID)
            self.Controller.controller.get_surrounding_vehicles = lambda *args, **kwargs: (None, [dummy_leader], None, None)

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
            #endregion
            self.APIControllerWrite(SpeedCMD, SteeringCMD)
            t = time.time()-t0
        self.APIControllerWrite(0, 0)
        pass

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