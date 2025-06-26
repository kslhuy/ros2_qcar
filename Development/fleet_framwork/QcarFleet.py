from pal.utilities.math import wrap_to_pi
from pal.products.qcar import QCar, QCarGPS, IS_PHYSICAL_QCAR
from pal.utilities.scope import MultiScope
import pal.resources.images as images

from qvl.qlabs import QuanserInteractiveLabs
from qvl.qcar2 import QLabsQCar2
from qvl.real_time import QLabsRealTime

from hal.content.qcar_functions import QCarEKF

from src.Controller.DummyController import DummyVehicle
from src.Controller.idm_control import IDMControl
from src.Controller.CACC import CACC
from src.OpenRoad import OpenRoad

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
    Qcars = []

    def __init__(self, NumQcar: int, LeaderIndex: int, Distance: float, Controller: str, Observer:str, QlabType:str = "OpenRoad"):
        """

        Controller/Observer     :str        - String The name in the 
        QlanType                :str        - Simulation Map
        Distance                :float      - The distance between the following position and the object car position 


        Function: Initiation and Build the fleet.
        """

        self.qlabs = QuanserInteractiveLabs()
        self.Qcars = []
        self.NumQcar = NumQcar
        self.LeaderIndex = LeaderIndex
        self.QcarIndexList = range(0, self.NumQcar)
        self.Distance = Distance            
        self.Controller = Controller        
        self.Observer = Observer            
        self.rtModel = os.path.normpath(os.path.join(os.environ['RTMODELS_DIR'], 'QCar2/QCar2_Workspace_studio'))
        self.InitEnv()
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
        pass


    #region: Initiation
    def InitEnv(self):
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
        #QLabsRealTime().terminate_all_real_time_models(RTModelHostName='host.docker.internal')
        pass

    def InitQcar(self, QlabType:str):
        """
        Generate the Qcar in the Qlab
        """
        for i in range(0, self.NumQcar):
            self.Qcars.append(QLabsQCar2(self.qlabs))

        match QlabType:
            case "OpenRoad":
                InitPositionTable = pd.read_csv("QcarDev/python/DO_1DimentionalCarFleet/data/QcarInitSettingOpenRoad.csv")

            case "Studio":
                InitPositionTable = pd.read_csv("QcarDev/python/DO_1DimentionalCarFleet/data/QcarInitSettingStudio.csv")
                
            case _:
                print("Error: QlabType not found")
                quit()

        InitPositionTable = InitPositionTable.to_numpy()
        for i in range(0, self.NumQcar):
            self.Qcars[i].spawn_id(actorNumber=i, location=InitPositionTable[i, 1:4], rotation=InitPositionTable[i,4:7], scale=[0.1,0.1,0.1])
        pass

    def InitThread(self):
        global KILL_THREAD
        KILL_THREAD = True
        pass
    #endregion


    #region: Main Program for the Fleet
    def FleetBuilding(self):
        """
        Start Following for Every Qcar in the Fleet
        """
        
        pass

    def FleetCanceling(self):
        """
        Cancel Following for Every Qcar in the Fleet
        """


        pass
    #endregion


    #region: API for writing the Fleet Leader and Get/Print Fleet Data
    def QcarInfoGet(self, CarIndex:int, InfoType:str = "position"):
        """
        Obtain the Data in current time for whole fleet or some qcar InformationType: all, position, rotation.
        """
        if CarIndex in self.QcarIndexList:
            pass
        else:
            print("Error: Illegal car index")
            quit()

        match InfoType:
            case "all":
                return self.Qcars[CarIndex].get_world_transform()
            case "angle":
                return self.Qcars[CarIndex].get_world_transform_degrees()
            case "exist":
                return self.Qcars[CarIndex].ping()
            case _:
                print("InfoType not in consideration, pls check")
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

        
        if InfoType == "all":
            if self.GetQcarInformation(CarIndex, "exist") == False:
                print("Qcar Index: ", CarIndex, "doesn't exist")
            else:
                print("Qcar Index: ", CarIndex, "exist", "Information:")
                print("   ","position: ", self.GetQcarInformation(CarIndex, "all")[1])
                print("   ","Angle: ", self.GetQcarInformation(CarIndex, "all")[2])
    #endregion