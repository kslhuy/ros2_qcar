import os, cv2, time, signal, threading
import numpy as np
import pyqtgraph as pg

from ControlThread import ControlThread

from pal.products.qcar import QCar, QCarGPS, IS_PHYSICAL_QCAR
from pal.utilities.math import wrap_to_pi
from hal.content.qcar_functions import QCarEKF

from src.OpenRoad import OpenRoad
from src.Controller.ControllerLeader import SpeedController,SteeringController

class ControlLeader(ControlThread):  
    def __init__(self,SimulationTime:float = 10, enableSteeringControl:bool = True, NodeSequence:list = [0,1], FlagPathRebuild:bool = False):
        super().__init__()                                  #  Initialize parent Thread class
        self.tf = SimulationTime
        self.startDelay = 1
        self.controllerUpdateRate = 100
        self.K_p = 0.2
        self.K_i = 1
        self.enableSteeringControl = enableSteeringControl
        self.K_stanley = 1
        self.calibrationPose = [0,2,-np.pi/2]
        self.calibrate = True
        print("Basic Setting Finished")
        self.waypointSequence, self.InitialPose = self.PathGeneration(FlagPathRebuild, NodeSequence)
        print("Map Generation Finished")

        self.speedController = SpeedController(
            kp=self.K_p,
            ki=self.K_i
        )
        if self.enableSteeringControl:
            self.steeringController = SteeringController(
                waypoints=self.waypointSequence,
                k=self.K_stanley
            )
        print("Controller Setting FInished")

    def run(self):
        qcar = QCar(readMode=1, frequency=self.controllerUpdateRate)
        if self.enableSteeringControl or self.calibrate:
            ekf = QCarEKF(x_0 = self.InitialPose)
            gps = QCarGPS(initialPose=self.calibrationPose, calibrate=self.calibrate)
        else:
            gps = memoryview(b'')

        with qcar, gps:
            t0 = time.time()
            t = 0
            delta = 0
            u = 0
            while t < self.tf + self.startDelay and not self._kill_thread.is_set():
                tp = t
                t = time.time() - t0
                dt = t - tp
                qcar.read()
                vref = self.vref(t)
                # vref = 120
                if self.enableSteeringControl:
                    if gps.readGPS():
                        y_gps = np.array([gps.position[0],
                                          gps.position[1],
                                          gps.orientation[2]])
                        ekf.update([qcar.motorTach, delta],
                                    dt,
                                    y_gps,
                                    qcar.gyroscope[2],)
                    else:
                        ekf.update([qcar.motorTach, delta],
                                    dt,
                                    None,
                                    qcar.gyroscope[2],)
                    x = ekf.x_hat[0, 0]
                    y = ekf.x_hat[1, 0]
                    th = ekf.x_hat[2, 0]
                    p = np.array([x, y]) + np.array([np.cos(th), np.sin(th)]) * 0.2
                else:
                    th = 0
                    p = None
                v = qcar.motorTach
                if t < self.startDelay:
                    u = 0
                    delta = 0
                else:
                    u = self.speedController.update(v, vref, dt)

                    if self.enableSteeringControl:
                        delta = self.steeringController.update(p, th, v)
                    else:
                        delta = 0
                qcar.write(u, delta)
            qcar.read_write_std(throttle=0, steering=0)
            print("Thread Ends: Leader Control")

    def PathGeneration(self, NeedRebuild:bool, NodeSequence:list):
        waypointSequenceLocation    = "Development\\fleet_framwork\\data\\InitialPose.npy"
        InitialPoseLocation         = "Development\\fleet_framwork\\data\\WayPintSequence.npy"
        if not NeedRebuild:
            if os.path.exists(waypointSequenceLocation):
                print("Path Exists, no need to rebuild")
                waypointSequence = np.load(waypointSequenceLocation)
                InitialPose = np.load(InitialPoseLocation)
            else:
                print("Path not Exist, rebuilding and save")
                roadmap = OpenRoad()
                waypointSequence = roadmap.generate_path(NodeSequence)
                InitialPose = roadmap.get_node_pose(NodeSequence[0]).squeeze()
                np.save(waypointSequenceLocation, waypointSequence)
                np.save(InitialPoseLocation, InitialPose)
        else:
            print("Path Exists, rebuilding")
            roadmap = OpenRoad()
            waypointSequence = roadmap.generate_path(NodeSequence)
            InitialPose = roadmap.get_node_pose(NodeSequence[0]).squeeze()
            np.save(waypointSequenceLocation, waypointSequence)
            np.save(InitialPoseLocation, InitialPose)
        print("Path Create/Load Complete")
        return waypointSequence, InitialPose

    def vref(self,t):
        if t<5:
            v_ref = 2
        elif t<10:
            v_ref = 0
        elif t<15:
            v_ref = -0.5
        elif t<20:
            v_ref = 1
        else:
            v_ref = 2
        return v_ref
