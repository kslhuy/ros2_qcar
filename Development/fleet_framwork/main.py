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
    # config = ConfigPresets.get_studio_cacc()
    config = ConfigPresets.get_openroad_cacc()
    
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



    Fleet = QcarFleet(QcarNum, LeaderIndex, DistanceBetweenEachCar, Controller, Observer , QlabType)
    print("Create Control Threading")
    LeaderControl = ControlLeader(SimTime, enableSteeringControl, NodeSequence, FlagPathRebuild , QlabType)
    
    # Pass configuration to ControlFollower instances
    FollowerControl = [ControlFollower(SimTime, Fleet, i, i - 1, config.lookahead_distance, config.max_steering, Controller, config) for i in range(1, QcarNum)]


    # Start followers
    for fc in FollowerControl:
        fc.start()
        time.sleep(0.1)

    # Start leader
    print("Leader Control Start")
    LeaderControl.start()
    

    # ------ Main thread to monitor the status of the threads ------
    try:
        print("Main Thread running")
        while not KILL_THREAD.is_set():
            all_dead = all(not fc.is_alive() for fc in FollowerControl) and not LeaderControl.is_alive()
            if all_dead:
                break
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("KeyboardInterrupt caught. Stopping threads...")

    finally:
        print("Stopping all threads...")
        LeaderControl.stop()
        for fc in FollowerControl:
            fc.stop()

        LeaderControl.join()
        for fc in FollowerControl:
            fc.join()

        print("Simulation Ends.")


if __name__ == "__main__":
    main()