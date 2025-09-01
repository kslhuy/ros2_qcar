import os, time
import signal
import multiprocessing

# Import the new simple configuration system
try:
    from simple_config import SimpleFleetConfig, ConfigPresets
    SIMPLE_CONFIG_AVAILABLE = True
    print("Using Simple YAML Configuration System (config.yaml)")
except ImportError:
    try:
        from simple_config_txt import SimpleFleetConfig, ConfigPresets
        SIMPLE_CONFIG_AVAILABLE = True
        print("Using Simple Text Configuration System (config.txt)")

    except ImportError:
        print("Warning: simple_config modules not available")
        print("Falling back to original FleetConfig system")
        from FleetConfig import FleetConfig, ConfigPresets, RoadType, ControllerType
        SIMPLE_CONFIG_AVAILABLE = False

from QcarFleet import QcarFleet
# from ControlLeader import ControlLeader
# from ControlFollower import ControlFollower


# Shared kill signal for main process
KILL_MAIN = False


def sig_handler(*args):
    global KILL_MAIN
    print("SIGINT received. Stopping all processes...")
    KILL_MAIN = True
signal.signal(signal.SIGINT, sig_handler)

def main():
    # =========================================================================
    # EASY CONFIGURATION LOADING
    # =========================================================================
    
    if SIMPLE_CONFIG_AVAILABLE:
        print("Using Simple Configuration System")
        # print("To modify settings, edit 'config.txt' file")
        
        # Option 1: Load from text file (recommended - easy to modify)
        # config = ConfigPresets.load_from_file("config.txt")
        config = ConfigPresets.load_from_file("config.yaml")

        
        # Option 2: Create preset configurations programmatically
        # config = ConfigPresets.create_studio_cacc()
        # config = ConfigPresets.create_openroad_cacc()
        
        # Option 3: Update specific parameters programmatically (optional)
        # config.update_config(**{
        #     'simulation.time': 60,
        #     'fleet.num_vehicles': 3,
        #     's0': 2.0,
        #     'ri': 2.0
        # })
        
    else:
        print("Using Original Configuration System")
        # Fallback to original system
        # config = ConfigPresets.get_openroad_cacc()
        config = ConfigPresets.get_studio_cacc()
        # config = ConfigPresets.get_openroad_cacc()
        
        # You can also update specific parameters
        # config.update_config(simulation_time=60, qcar_num=3, s0=5, ri=5)
    
    # Print current configuration
    print("\nCurrent Configuration:")
    config.print_config()
    
    # Extract parameters from config (same interface as before)
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

    # Create fleet with new process-based architecture
    Fleet = QcarFleet(QcarNum, LeaderIndex, DistanceBetweenEachCar, Controller, Observer, QlabType, config)
    
    print("Fleet created with process-based vehicle architecture")
    print(f"Number of vehicles: {QcarNum}")
    print(f"Leader index: {LeaderIndex}")
    print(f"Controller type: {Controller}")
    
    # Start all vehicle processes
    print("Starting fleet vehicle processes...")
    Fleet.FleetBuilding()

    # ------ Main process to monitor the status of the vehicle processes ------
    try:
        print("Main Process running - monitoring fleet status")
        start_time = time.time()
        last_status_time = start_time

        
        while not KILL_MAIN:
            current_time = time.time()
            
            # Check if simulation time limit reached
            if SimTime > 0 and (current_time - start_time) >= SimTime:
                print(f"Simulation time limit ({SimTime}s) reached")
                break
            
            # Check if all vehicle processes are still alive
            if not Fleet.is_fleet_alive():
                print("All vehicle processes have stopped")
                break
                
            # Print fleet status every 5 seconds
            if current_time - last_status_time >= 5:
                status = Fleet.get_fleet_status()
                alive_count = sum(1 for v in status.values() if v['alive'])
                print(f"Fleet status: {alive_count}/{QcarNum} vehicle processes alive")
                
                # Print any vehicle status updates from the queue
                try:
                    while True:
                        update = Fleet.status_queue.get_nowait()
                        if update['status'] == 'running':
                            pos = update.get('position', [0, 0, 0])
                            vel = update.get('velocity', 0)
                            print(f"  Vehicle {update['vehicle_id']}: pos=({pos[0]:.2f}, {pos[1]:.2f}), vel={vel:.2f}")
                except:
                    pass  # Queue is empty
                    
                last_status_time = current_time

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("KeyboardInterrupt caught. Stopping fleet...")

    finally:
        print("Stopping all vehicle processes...")
        Fleet.FleetCanceling()

        print("Simulation Ends.")


if __name__ == "__main__":
    # Set multiprocessing start method for Windows compatibility
    multiprocessing.set_start_method('spawn', force=True)
    main()