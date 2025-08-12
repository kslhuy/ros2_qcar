import os, time
import signal
import threading

from FleetConfig import FleetConfig, ConfigPresets, RoadType, ControllerType
from QcarFleet import QcarFleet
from ControlLeader import ControlLeader
from ControlFollower import ControlFollower


# Shared kill signal for all threads
KILL_THREAD = threading.Event()


def sig_handler(*args):
    print("SIGINT received. Stopping all threads...")
    KILL_THREAD.set()
signal.signal(signal.SIGINT, sig_handler)

def main():
    # Load configuration - you can easily change this to different presets
    # config = ConfigPresets.get_openroad_cacc()
    config = ConfigPresets.get_studio_cacc()
    # config = ConfigPresets.get_openroad_cacc()
    
    # Or create custom configuration
    # config = FleetConfig(RoadType.OpenRoad, ControllerType.CACC)
    
    # You can also update specific parameters
    # config.update_config(simulation_time=60, qcar_num=3, s0=5, ri=5)
    
    # Print current configuration
    config.print_config()
    
    # Extract parameters from config
    SimTime = config.simulation_time
    LeaderIndex = config.leader_index
    QcarNum = config.qcar_num
    DistanceBetweenEachCar = config.distance_between_cars
    Controller = config.get_controller_type_name()
    Observer = ""
    QlabType = config.get_road_type_name()
    NodeSequence = config.get_node_sequence()
    enableSteeringControl = config.enable_steering_control
    FlagPathRebuild = config.flag_path_rebuild

    # Create fleet with new architecture - pass config to fleet
    Fleet = QcarFleet(QcarNum, LeaderIndex, DistanceBetweenEachCar, Controller, Observer, QlabType, config)
    
    print("Fleet created with Vehicle instances")
    print(f"Number of vehicles: {QcarNum}")
    print(f"Leader index: {LeaderIndex}")
    print(f"Controller type: {Controller}")
    
    # Start all vehicles using the new fleet method
    print("Starting fleet vehicles...")
    Fleet.FleetBuilding()
    
    # Optional: Start leader control if you still want separate leader control
    # This is now redundant since vehicles manage themselves
    # LeaderControl = ControlLeader(SimTime, enableSteeringControl, NodeSequence, FlagPathRebuild , QlabType)
    # LeaderControl.start()

    # ------ Main thread to monitor the status of the vehicles ------
    try:
        print("Main Thread running - monitoring fleet status")
        start_time = time.time()
        last_status_time = start_time

        
        while not KILL_THREAD.is_set():
            current_time = time.time()
            
            # Check if simulation time limit reached
            if SimTime > 0 and (current_time - start_time) >= SimTime:
                print(f"Simulation time limit ({SimTime}s) reached")
                break
            
            # Check if all vehicles are still alive
            if not Fleet.is_fleet_alive():
                print("All vehicles have stopped")
                break
                
            # Print fleet status every 5 seconds
            if current_time - last_status_time >= 5:
                status = Fleet.get_fleet_status()
                alive_count = sum(1 for v in status.values() if v['alive'])
                print(f"Fleet status: {alive_count}/{QcarNum} vehicles alive")
                last_status_time = current_time

            
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("KeyboardInterrupt caught. Stopping fleet...")

    finally:
        print("Stopping all vehicles...")
        Fleet.FleetCanceling()
        
        # if 'LeaderControl' in locals():
        #     LeaderControl.stop()
        #     LeaderControl.join()

        print("Simulation Ends.")


if __name__ == "__main__":
    main()