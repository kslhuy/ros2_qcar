from pal.utilities.math import wrap_to_pi
from pal.products.qcar import QCar, QCarGPS, IS_PHYSICAL_QCAR
from pal.utilities.scope import MultiScope
import pal.resources.images as images

from qvl.qlabs import QuanserInteractiveLabs
from qvl.qcar2 import QLabsQCar2
from qvl.real_time import QLabsRealTime
#Environment
from qvl.basic_shape import QLabsBasicShape
from qvl.walls import QLabsWalls
from qvl.qcar_flooring import QLabsQCarFlooring
from qvl.crosswalk import QLabsCrosswalk

from hal.content.qcar_functions import QCarEKF

from src.Controller.DummyController import DummyVehicle
from src.Controller.idm_control import IDMControl
from src.Controller.CACC import CACC
from src.OpenRoad import OpenRoad
from Vehicle import Vehicle

import time, io, math, threading, os, multiprocessing
import numpy as np
import pandas as pd

class QcarFleet:
    """ QCarFleet Class
    
    :NumQcar: Number of Qcars in the Fleet
    :Distance: the distance between the following point and the target car for each follower car.
    :Controller: The controller used for each car model
    :Observer: The observer used for the fleet.

    :QcarIndexList: List of Qcar Index, from 0 to (NumQCar-1)
    """

    def __init__(self, NumQcar: int, LeaderIndex: int, Distance: float, Controller: str, Observer:str, QlabType:str = "OpenRoad", config=None):
        """
        Controller/Observer     :str        - String The name in the 
        QlanType                :str        - Simulation Map
        Distance                :float      - The distance between the following position and the object car position 
        config                  :object     - Configuration object containing fleet parameters

        Function: Initiation and Build the fleet.
        
        Note: NumQcar supports up to 5 vehicles (modular design), but typically 3 vehicles are used.
        Each vehicle gets assigned specific ports for socket communication on localhost.
        """

        self.qlabs = QuanserInteractiveLabs()
        self.Qcars = []  # This will now store Vehicle instances
        self.qcar_objects = []  # Store raw QLabsQCar2 objects
        self.NumQcar = NumQcar
        self.LeaderIndex = LeaderIndex
        self.QcarIndexList = range(0, self.NumQcar)
        self.Distance = Distance            
        self.Controller = Controller        
        self.Observer = Observer
        self.config = config
        self.rtModel = os.path.normpath(os.path.join(os.environ['RTMODELS_DIR'], 'QCar2/QCar2_Workspace_studio'))
        self.InitEnv(QlabType)
        #Number of the Qcars in the fleet, int
        if self.NumQcar < 2:
            print("Error: Number of cars in the fleet is too small")
            quit()
            #Check the number of cars in the fleet. 
        self.InitQcar(QlabType)                 #Generate the Qcars.

        if not self.LeaderIndex in self.QcarIndexList:
            print("Error: Leader Car Index Illegal")
            quit()
        else:
            QLabsRealTime().start_real_time_model(self.rtModel, actorNumber=self.LeaderIndex)

        self.InitThread()
        self.InitVehicles()  # Initialize Vehicle instances
        pass


    #region: Initiation
    def InitEnv(self , QlabType:str):
        """
        Initiate the environment of the Qlab 
        """
        try:
            self.qlabs.open("localhost")
            #qlabs.open("host.docker.internal")
            print("Connected to QLabs")
        except:
            print("Error: Unable to connect to QLabs")
            quit()
        self.qlabs.destroy_all_spawned_actors()
        QLabsRealTime().terminate_all_real_time_models()

        if (QlabType == "Studio"):
            # Setup environment
            x_offset = 0.13
            y_offset = 1.67
            hFloor = QLabsQCarFlooring(self.qlabs)
            hFloor.spawn_degrees([x_offset, y_offset, 0.001], rotation=[0, 0, -90], configuration=0)
            hWall = QLabsWalls(self.qlabs)
            hWall.set_enable_dynamics(False)
            for y in range(5):
                hWall.spawn_degrees(location=[-2.4 + x_offset, (-y*1.0)+2.55 + y_offset, 0.001], rotation=[0, 0, 0])
            for x in range(5):
                hWall.spawn_degrees(location=[-1.9+x + x_offset, 3.05+ y_offset, 0.001], rotation=[0, 0, 90])
            for y in range(6):
                hWall.spawn_degrees(location=[2.4+ x_offset, (-y*1.0)+2.55 + y_offset, 0.001], rotation=[0, 0, 0])
            for x in range(4):
                hWall.spawn_degrees(location=[-0.9+x+ x_offset, -3.05+ y_offset, 0.001], rotation=[0, 0, 90])
            hWall.spawn_degrees(location=[-2.03 + x_offset, -2.275+ y_offset, 0.001], rotation=[0, 0, 48])
            hWall.spawn_degrees(location=[-1.575+ x_offset, -2.7+ y_offset, 0.001], rotation=[0, 0, 48])
            myCrossWalk = QLabsCrosswalk(self.qlabs)
            myCrossWalk.spawn_degrees(location=[-2 + x_offset, -1.475 + y_offset, 0.01], rotation=[0, 0, 0], scale=[0.1, 0.1, 0.075], configuration=0)
            mySpline = QLabsBasicShape(self.qlabs)
            mySpline.spawn_degrees(location=[2.05 + x_offset, -1.5 + y_offset, 0.01], rotation=[0, 0, 0], scale=[0.27, 0.02, 0.001], waitForConfirmation=False)


        #QLabsRealTime().terminate_all_real_time_models(RTModelHostName='host.docker.internal')
        pass

    def InitQcar(self, QlabType:str):
        """
        Generate the Qcar in the Qlab
        """

        # import os
        for i in range(0, self.NumQcar):
            self.qcar_objects.append(QLabsQCar2(self.qlabs))

        match QlabType:
            case "OpenRoad":
                base_dir = os.path.dirname(__file__)
                csv_path = os.path.join(base_dir, "data", "QcarInitSettingOpenRoad.csv")
                InitPositionTable = pd.read_csv(csv_path)
                QcarScale = [1,1,1] 

            case "Studio":
                base_dir = os.path.dirname(__file__)
                csv_path = os.path.join(base_dir, "data", "QcarInitSettingStudio.csv")
                InitPositionTable = pd.read_csv(csv_path)
                QcarScale =  [0.1,0.1,0.1]
            case _:
                QcarScale = [1,1,1]
                print("Error: QlabType not found")
                quit()

        InitPositionTable = InitPositionTable.to_numpy()
        # QcarScale = [1,1,1] 
        #Real scale for simulation and physical qcar: 
        for i in range(0, self.NumQcar):
            self.qcar_objects[i].spawn_id(actorNumber=i, location=InitPositionTable[i, 1:4], rotation=InitPositionTable[i,4:7], scale=QcarScale)
        pass

    def InitThread(self):
        self.lock = threading.Lock()
        pass

    def InitVehicles(self):
        """
        Initialize Vehicle instances for each QCar with specific port assignments
        Communication chain: Leader sends to followers, followers receive from leader
        """
        # Port configuration for up to 5 vehicles (modular design)
        # Format: [send_port, recv_port, ack_port] for each vehicle
        # Communication design: Leader broadcasts to all followers
        # Leader sends on 6001 -> All followers receive on 6001
        port_config = {
            0: [6001, 6000, 6002],  # Vehicle 0 (Leader): sends on 6001, receives on 6000
            1: [6000, 6001, 6012],  # Vehicle 1 (Follower): sends on 6000, receives on 6001 (from leader)
            2: [6021, 6001, 6022],  # Vehicle 2 (Follower): sends on 6021, receives on 6001 (from leader)  
            3: [6031, 6001, 6032],  # Vehicle 3 (   Follower): sends on 6031, receives on 6001 (from leader) - optional
            4: [6041, 6001, 6042],  # Vehicle 4 (Follower): sends on 6041, receives on 6001 (from leader) - optional
        }
        
        # Validate NumQcar doesn't exceed our port configuration
        max_vehicles = len(port_config)
        if self.NumQcar > max_vehicles:
            print(f"Error: Number of vehicles ({self.NumQcar}) exceeds maximum supported ({max_vehicles})")
            print(f"Reducing NumQcar to {max_vehicles}")
            self.NumQcar = max_vehicles
            self.QcarIndexList = range(0, self.NumQcar)
        
        for i in range(self.NumQcar):
            is_leader = (i == self.LeaderIndex)
            send_port, recv_port, ack_port = port_config[i]

            vehicle = Vehicle(
                vehicle_id=i,
                qcar=self.qcar_objects[i],
                controller_type=self.Controller,
                is_leader=is_leader,
                config=self.config,
                fleet_lock=self.lock,
                target_ip="127.0.0.1",  # localhost for local testing
                base_send_port=send_port,
                base_recv_port=recv_port,
                base_ack_port=ack_port
            )
            self.Qcars.append(vehicle)
            print(f"Vehicle {i} {'(Leader)' if is_leader else '(Follower)'}: "
                  f"Send={send_port}, Recv={recv_port}, ACK={ack_port}")
        
        # Set leader-follower relationships
        leader_vehicle = self.Qcars[self.LeaderIndex]
        for i, vehicle in enumerate(self.Qcars):
            if not vehicle.is_leader:
                vehicle.set_leader(leader_vehicle)
        
        print(f"Initialized {self.NumQcar} vehicles with leader at index {self.LeaderIndex}")
        print("Communication setup: Leader broadcasts on port 6001, all followers listen on port 6001")
        print("Port configuration complete for local socket communication")
        pass
    #endregion


    #region: Main Program for the Fleet
    def FleetBuilding(self):
        """
        Start Following for Every Qcar in the Fleet
        """
        # print("Starting fleet vehicles...")
        for i, vehicle in enumerate(self.Qcars):
            vehicle.start()
            time.sleep(0.1)  # Small delay between starting vehicles
            # print(f"Started vehicle {i} ({'Leader' if vehicle.is_leader else 'Follower'})")
        print("All fleet vehicles started")
        pass

    def FleetCanceling(self):
        """
        Cancel Following for Every Qcar in the Fleet
        """
        print("Stopping fleet vehicles...")
        for i, vehicle in enumerate(self.Qcars):
            vehicle.stop()
            print(f"Stopped vehicle {i}")
        
        # Wait for all vehicles to finish
        for vehicle in self.Qcars:
            vehicle.join(timeout=2.0)
        
        print("All fleet vehicles stopped")
        pass

    def get_fleet_status(self):
        """
        Get status of all vehicles in the fleet
        """
        status = {}
        for i, vehicle in enumerate(self.Qcars):
            status[i] = {
                'alive': vehicle.is_alive(),
                'state': vehicle.get_state()
            }
        return status

    def is_fleet_alive(self):
        """
        Check if any vehicle in the fleet is still running
        """
        return any(vehicle.is_alive() for vehicle in self.Qcars)
    #endregion


    #region: API for writing the Fleet Leader and Get/Print Fleet Data
    def APIQcarInfoGet(self, CarIndex:int, InfoType:str = "position"):
        """
        Obtain the Data in current time for whole fleet or some qcar InformationType: all, position, rotation.
        
        CarIndex            :str                The index of the object Qcar
        InfoType            :position           The requirement information ("all", "position", "rotation")
        """
        if CarIndex in self.QcarIndexList:
            pass
        else:
            print("Error: Illegal car index")
            quit()

        vehicle = self.Qcars[CarIndex]
        
        match InfoType:
            case "all":
                return vehicle.qcar.get_world_transform()
            case "angle":
                return vehicle.qcar.get_world_transform_degrees()
            case "exist":
                return vehicle.qcar.ping()
            case "state":
                return vehicle.get_state()
            case _:
                print("InfoType not in consideration, pls check")
        pass

    def APIQcarWrite(self, CarIndex:int, SpeedCMD:float = 0, SteeringCMD:float = 0 ):
        if CarIndex in self.QcarIndexList:
            pass
        else:
            print("Error: illegal car index")
            quit()

        vehicle = self.Qcars[CarIndex]
        vehicle.qcar.set_velocity_and_request_state(
                    forward         =SpeedCMD,
                    turn            =SteeringCMD,
                    headlights      =False,
                    leftTurnSignal  =False,
                    rightTurnSignal =False,
                    brakeSignal     =False,
                    reverseSignal   =False
                )
        pass

    def QcarInfoPrint(self, CarIndex:int, InfoType:str = "all"):
        """
        Qcar Data API
        """
        if CarIndex in self.QcarIndexList:
            pass
        else:
            print("Error: illegal car index")
            quit()

        vehicle = self.Qcars[CarIndex]
        
        if InfoType == "all":
            if not vehicle.qcar.ping():
                print("Qcar Index: ", CarIndex, "doesn't exist")
            else:
                print("Qcar Index: ", CarIndex, "exist", "Information:")
                print("   ","position: ", vehicle.current_pos)
                print("   ","Angle: ", vehicle.current_rot)
                print("   ","Velocity: ", vehicle.velocity)
                print("   ","Running: ", vehicle.is_alive())
        elif InfoType == "state":
            state = vehicle.get_state()
            print(f"Vehicle {CarIndex} State: {state}")

    
    #endregion