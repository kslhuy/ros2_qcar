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
from VehicleProcess import vehicle_process_main

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
        self.vehicle_processes = []  # Store process objects instead of Vehicle instances
        self.vehicle_configs = []   # Store vehicle configurations for processes
        self.NumQcar = NumQcar
        self.LeaderIndex = LeaderIndex
        self.QcarIndexList = range(0, self.NumQcar)
        self.Distance = Distance            
        self.Controller = Controller        
        self.Observer = Observer
        self.config = config
        
        # Multiprocessing setup
        self.stop_event = multiprocessing.Event()
        self.status_queue = multiprocessing.Queue()
        
        self.InitEnv(QlabType)
        #Number of the Qcars in the fleet, int
        if self.NumQcar < 2:
            print("Error: Number of cars in the fleet is too small")
            quit()
            #Check the number of cars in the fleet. 
        self.InitQcarSpawnData(QlabType)  # Only prepare spawn data, don't spawn here
        
        # if not self.LeaderIndex in self.QcarIndexList:
        #     print("Error: Leader Car Index Illegal")
        #     quit()
        # else:
        #     QLabsRealTime().start_real_time_model(self.rtModel, actorNumber=self.LeaderIndex)

        self.InitThread()  # Keep for compatibility, but won't use threading locks
        self.PrepareVehicleConfigs()  # Prepare configurations for vehicle processes
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

    def InitQcarSpawnData(self, QlabType:str):
        """
        Prepare spawn data for Qcars - actual spawning will happen in each process
        """

        match QlabType:
            case "OpenRoad":
                base_dir = os.path.dirname(__file__)
                csv_path = os.path.join(base_dir, "data", "QcarInitSettingOpenRoad.csv")
                InitPositionTable = pd.read_csv(csv_path)
                self.QcarScale = [1,1,1] 

            case "Studio":
                base_dir = os.path.dirname(__file__)
                csv_path = os.path.join(base_dir, "data", "QcarInitSettingStudio.csv")
                InitPositionTable = pd.read_csv(csv_path)
                self.QcarScale =  [0.1,0.1,0.1]
            case _:
                self.QcarScale = [1,1,1]
                print("Error: QlabType not found")
                quit()

        InitPositionTable = InitPositionTable.to_numpy()
        
        # Store the position table for use during vehicle process initialization
        self.InitPositionTable = InitPositionTable
        
        print(f"Prepared spawn data for {self.NumQcar} vehicles with scale {self.QcarScale}")
        pass

    def InitThread(self):
        self.lock = threading.Lock()
        pass

    def PrepareVehicleConfigs(self):
        """
        Prepare configuration dictionaries for each vehicle process.
        Each process will create its own QLabs connection and spawn its vehicle.
        
        NEW DESIGN: Bidirectional communication where each vehicle can communicate with all others
        Each vehicle has its own unique send/receive ports for peer-to-peer communication
        """
        # Port configuration for up to 5 vehicles (bidirectional design)
        # Format: [send_port, recv_port, ack_port] for each vehicle
        # NEW: Each vehicle has unique ports for bidirectional communication
        port_config = {
            0: [6000, 6000, 6002],  # Vehicle 0: send=6000, recv=6000, ack=6002
            1: [6010, 6010, 6012],  # Vehicle 1: send=6010, recv=6010, ack=6012
            2: [6020, 6020, 6022],  # Vehicle 2: send=6020, recv=6020, ack=6022  
            3: [6030, 6030, 6032],  # Vehicle 3: send=6030, recv=6030, ack=6032
            4: [6040, 6040, 6042],  # Vehicle 4: send=6040, recv=6040, ack=6042
        }
        
        # Create peer port mapping for bidirectional communication
        # Each vehicle knows all other vehicles' ports for direct communication
        self.peer_ports = {}
        for vehicle_id in range(self.NumQcar):
            self.peer_ports[vehicle_id] = {}
            for peer_id in range(self.NumQcar):
                if peer_id != vehicle_id:
                    peer_send_port, peer_recv_port, peer_ack_port = port_config[peer_id]
                    self.peer_ports[vehicle_id][peer_id] = {
                        'send_to_peer': peer_recv_port,  # Send to peer's receive port
                        'recv_from_peer': peer_send_port, # Receive from peer's send port
                        'ack_from_peer': peer_ack_port    # Acknowledge to peer's ack port
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
            
            # NEW: Chain-following configuration
            # Each vehicle follows the one directly in front of it in the convoy
            if is_leader:
                following_target = None  # Leader doesn't follow anyone
            else:
                following_target = i - 1  # Follow the vehicle with ID one less than mine
            
            print(f"Vehicle {i}: {'Leader' if is_leader else f'Follower (following vehicle {following_target})'}")
            
            # Extract initial pose for this vehicle from InitPositionTable
            # InitPositionTable format: [QcarIndex, PositionX, PositionY, PositionZ, RotationX, RotationY, RotationZ]
            vehicle_initial_pose = None
            spawn_location = [0, 0, 0]
            spawn_rotation = [0, 0, 0]
            
            if hasattr(self, 'InitPositionTable') and i < len(self.InitPositionTable):
                # Convert from CSV format to [x, y, z, roll, pitch, yaw]
                pos_row = self.InitPositionTable[i]
                vehicle_initial_pose = [
                    pos_row[1],  # PositionX
                    pos_row[2],  # PositionY
                    pos_row[3],  # PositionZ
                    pos_row[4],  # RotationX
                    pos_row[5],  # RotationY
                    pos_row[6]   # RotationZ
                ]
                spawn_location = [pos_row[1], pos_row[2], pos_row[3]]
                spawn_rotation = [pos_row[4], pos_row[5], pos_row[6]]
                print(f"Vehicle {i}: Initial pose set to x={vehicle_initial_pose[0]:.3f}, "
                      f"y={vehicle_initial_pose[1]:.3f}, yaw={vehicle_initial_pose[5]:.3f}")
            else:
                print(f"Vehicle {i}: No initial pose data available, using default")

            # Create configuration dictionary for this vehicle process
            vehicle_config = {
                'vehicle_id': i,
                'controller_type': self.Controller,
                'is_leader': is_leader,
                'fleet_size': self.NumQcar,
                'initial_pose': vehicle_initial_pose,
                
                # NEW: Chain-following configuration
                'following_target': following_target,  # Which vehicle this one should follow
                
                # NEW: Bidirectional communication settings
                'target_ip': "127.0.0.1",  # localhost for local testing
                'send_port': send_port,
                'recv_port': recv_port,
                'ack_port': ack_port,
                'peer_ports': self.peer_ports[i],  # Peer communication mapping
                'communication_mode': 'bidirectional',  # Enable bidirectional mode
                
                # GPS settings
                'gps_server_ip': "127.0.0.1",
                'gps_server_port': 8001,
                
                # Control parameters (extract from config object)
                'max_steering': getattr(self.config, 'max_steering', 0.6),
                'lookahead_distance': getattr(self.config, 'lookahead_distance', 7.0),
                'update_rate': 100,  # Hz
                'observer_rate': 100,  # Hz
                'gps_update_rate': 50,  # Hz


                # Configure observer to use EKF (like vehicle_control2.py)
                'observer': {
                    'local_observer_type': 'kalman',  # # "kalman", "luenberger", or "direct"
                    'enable_distributed': True,
                    "distributed_observer_type": "consensus",
                    'enable_noise_measurement': False,
                    'enable_prediction': True,
                    'consensus_gain': 0.1,
                },
                
                # Vehicle spawn information
                'vehicle_scale': self.QcarScale,
                'spawn_location': spawn_location,
                'spawn_rotation': spawn_rotation,
                
                # Fleet information
                'leader_index': self.LeaderIndex,
                'distance_between_cars': self.Distance,
                
                # Additional config parameters (convert config object to dict)
                'simulation_time': getattr(self.config, 'simulation_time', 0),
                'enable_steering_control': getattr(self.config, 'enable_steering_control', True),
                'road_type': self.config.get_road_type_name() if self.config else 'OpenRoad',
                'controller_type': self.config.get_controller_type_name() if self.config else 'CACC',
                'node_sequence': getattr(self.config, 'node_sequence', [0, 1]),
                'dummy_controller_params': getattr(self.config, 'dummy_controller_params', {}),

                #Vehicle Physic or virtual (use realtime model) 
                
                'use_physical_qcar': getattr(self.config, 'use_physical_qcar', False),
                'use_control_observer_mode' : getattr(self.config, 'use_control_observer_mode', True)

            }


            self.vehicle_configs.append(vehicle_config)
            print(f"Vehicle {i} {'(Leader)' if is_leader else '(Follower)'}: "
                  f"Send={send_port}, Recv={recv_port}, ACK={ack_port}")
        
        print(f"Prepared configurations for {self.NumQcar} vehicles with leader at index {self.LeaderIndex}")
        print("NEW: Bidirectional communication setup - Each vehicle can communicate with all others")
        print("Port configuration complete for bidirectional peer-to-peer communication")
        print(f"Peer port mapping: {self.peer_ports}")
        pass
    #endregion


    #region: Main Program for the Fleet
    def FleetBuilding(self):
        """
        Start processes for every vehicle in the fleet
        """
        print("Starting fleet vehicle processes...")
        
        for i, vehicle_config in enumerate(self.vehicle_configs):
            # Create a new process for each vehicle
            process = multiprocessing.Process(
                target=vehicle_process_main,
                args=(vehicle_config, self.stop_event, self.status_queue),
                name=f"Vehicle-{i}"
            )
            process.start()
            self.vehicle_processes.append(process)
            time.sleep(0.2)  # Small delay between starting processes
            print(f"Started process for vehicle {i} ({'Leader' if vehicle_config['is_leader'] else 'Follower'})")
        
        # Wait for all vehicles to initialize
        print("Waiting for vehicle processes to initialize...")
        initialized_count = 0
        timeout = 30.0  # 30 second timeout
        start_time = time.time()
        
        while initialized_count < self.NumQcar and (time.time() - start_time) < timeout:
            try:
                status = self.status_queue.get(timeout=1.0)
                if status['status'] == 'initialized':
                    initialized_count += 1
                    print(f"Vehicle {status['vehicle_id']} initialized ({initialized_count}/{self.NumQcar})")
                elif status['status'] == 'failed':
                    print(f"Vehicle {status['vehicle_id']} failed to initialize: {status.get('error', 'Unknown error')}")
            except:
                pass  # Timeout on queue get
        
        if initialized_count == self.NumQcar:
            print(f"All {self.NumQcar} fleet vehicle processes started and initialized successfully")
        else:
            print(f"Warning: Only {initialized_count}/{self.NumQcar} vehicles initialized within timeout")
        pass

    def FleetCanceling(self):
        """
        Stop all vehicle processes in the fleet
        """
        print("Stopping fleet vehicle processes...")
        
        # Signal all processes to stop gracefully
        self.stop_event.set()
        
        # Give processes time to stop gracefully
        print("Waiting for vehicle processes to stop gracefully...")
        time.sleep(2.0)  # Allow 2 seconds for graceful shutdown
        
        # Wait for processes to finish gracefully
        timeout = 3.0  # 3 second timeout per process
        for i, process in enumerate(self.vehicle_processes):
            if process.is_alive():
                print(f"Waiting for vehicle {i} process to stop...")
                process.join(timeout)
                
                if process.is_alive():
                    print(f"Vehicle {i} process did not stop gracefully, terminating...")
                    process.terminate()
                    process.join(timeout=2.0)
                    
                    if process.is_alive():
                        print(f"Vehicle {i} process did not terminate, killing...")
                        process.kill()
                        process.join()
                
                print(f"Vehicle {i} process stopped")
        
        # Clear the process list
        self.vehicle_processes.clear()
        
        # Close QLabs connection in main process
        try:
            if hasattr(self, 'qlabs') and self.qlabs is not None:
                print("Closing main QLabs connection...")
                self.qlabs.close()
                print("Main QLabs connection closed")
        except Exception as e:
            print(f"Error closing QLabs connection: {e}")
        
        # Terminate any remaining real-time models
        try:
            from qvl.real_time import QLabsRealTime
            print("Terminating all real-time models...")
            QLabsRealTime().terminate_all_real_time_models()
            print("All real-time models terminated")
        except Exception as e:
            print(f"Error terminating real-time models: {e}")
        
        print("All fleet vehicle processes stopped")
        pass

    def get_fleet_status(self):
        """
        Get status of all vehicles in the fleet by checking their processes
        and querying the status queue
        """
        status = {}
        
        # Check process status
        for i, process in enumerate(self.vehicle_processes):
            status[i] = {
                'alive': process.is_alive(),
                'pid': process.pid if process.is_alive() else None,
                'state': 'running' if process.is_alive() else 'stopped'
            }
        
        # Get any recent status updates from the queue
        recent_updates = {}
        try:
            while True:
                update = self.status_queue.get_nowait()
                vehicle_id = update['vehicle_id']
                recent_updates[vehicle_id] = update
        except:
            pass  # Queue is empty
        
        # Merge recent updates into status
        for vehicle_id, update in recent_updates.items():
            if vehicle_id in status:
                status[vehicle_id].update(update)
        
        return status

    def is_fleet_alive(self):
        """
        Check if any vehicle process in the fleet is still running
        """
        return any(process.is_alive() for process in self.vehicle_processes)

    def monitor_fleet_status(self, interval=5.0):
        """
        Monitor and print fleet status periodically
        """
        try:
            while not self.stop_event.is_set() and self.is_fleet_alive():
                status = self.get_fleet_status()
                alive_count = sum(1 for v in status.values() if v['alive'])
                print(f"Fleet status: {alive_count}/{self.NumQcar} vehicles alive")
                
                # Print any status updates from queue
                try:
                    while True:
                        update = self.status_queue.get_nowait()
                        if update['status'] == 'running':
                            pos = update.get('position', [0, 0, 0])
                            vel = update.get('velocity', 0)
                            print(f"Vehicle {update['vehicle_id']}: pos=({pos[0]:.2f}, {pos[1]:.2f}), vel={vel:.2f}")
                except:
                    pass  # Queue is empty
                
                time.sleep(interval)
        except KeyboardInterrupt:
            print("Fleet monitoring interrupted")
    #endregion


    #region: API for writing the Fleet Leader and Get/Print Fleet Data
    def APIQcarInfoGet(self, CarIndex:int, InfoType:str = "position"):
        """
        Obtain the Data in current time for whole fleet or some qcar InformationType: all, position, rotation.
        
        CarIndex            :int                The index of the object Qcar
        InfoType            :str                The requirement information ("all", "position", "rotation", "state", "exist")
        
        Note: In process-based architecture, this method has limited functionality.
        Vehicle data is primarily accessible through the status queue.
        """
        if CarIndex not in self.QcarIndexList:
            print("Error: Illegal car index")
            return None

        # Check if process exists and is alive
        if CarIndex < len(self.vehicle_processes):
            process = self.vehicle_processes[CarIndex]
            
            match InfoType:
                case "exist":
                    return process.is_alive()
                case "state":
                    # Try to get recent state from status queue
                    try:
                        recent_states = {}
                        while True:
                            status = self.status_queue.get_nowait()
                            recent_states[status['vehicle_id']] = status
                    except:
                        pass
                    
                    return recent_states.get(CarIndex, {'status': 'unknown'})
                case "all" | "position" | "rotation":
                    print(f"Warning: {InfoType} data not directly available in process-based architecture")
                    print("Use get_fleet_status() or monitor status queue for vehicle data")
                    return None
                case _:
                    print("InfoType not supported in process-based architecture")
                    return None
        else:
            print(f"Error: Vehicle {CarIndex} process not found")
            return None

    def APIQcarWrite(self, CarIndex:int, SpeedCMD:float = 0, SteeringCMD:float = 0):
        """
        Write commands to a vehicle.
        
        Note: Direct vehicle control is not available in process-based architecture.
        Each vehicle process manages its own control loop independently.
        """
        print("Warning: Direct vehicle control not available in process-based architecture")
        print("Vehicle processes manage their own control loops based on their configuration")
        print(f"Requested: Vehicle {CarIndex}, Speed={SpeedCMD}, Steering={SteeringCMD}")
        return False

    def QcarInfoPrint(self, CarIndex:int, InfoType:str = "all"):
        """
        Print Qcar Data
        
        Note: In process-based architecture, this prints process status and recent queue data.
        """
        if CarIndex not in self.QcarIndexList:
            print("Error: illegal car index")
            return

        if CarIndex < len(self.vehicle_processes):
            process = self.vehicle_processes[CarIndex]
            
            if InfoType == "all":
                if not process.is_alive():
                    print(f"Vehicle {CarIndex}: Process not running (PID: {process.pid})")
                else:
                    print(f"Vehicle {CarIndex}: Process running (PID: {process.pid})")
                    
                    # Try to get recent data from status queue
                    try:
                        recent_data = None
                        temp_data = []
                        while True:
                            status = self.status_queue.get_nowait()
                            temp_data.append(status)
                            if status['vehicle_id'] == CarIndex:
                                recent_data = status
                        
                        # Put back all the data we took out
                        for data in temp_data:
                            self.status_queue.put(data)
                        
                        if recent_data:
                            pos = recent_data.get('position', [0, 0, 0])
                            vel = recent_data.get('velocity', 0)
                            timestamp = recent_data.get('timestamp', 0)
                            print(f"   Recent data: position: {pos}")
                            print(f"   Velocity: {vel}")
                            print(f"   Timestamp: {timestamp}")
                            print(f"   Status: {recent_data.get('status', 'unknown')}")
                        else:
                            print(f"   No recent data available from status queue")
                    except:
                        print(f"   No data available in status queue")
                        
            elif InfoType == "state":
                # Get process state
                state = {
                    'process_alive': process.is_alive(),
                    'process_pid': process.pid if process.is_alive() else None,
                    'vehicle_id': CarIndex
                }
                print(f"Vehicle {CarIndex} Process State: {state}")
        else:
            print(f"Error: Vehicle {CarIndex} process not found")
    
    #endregion