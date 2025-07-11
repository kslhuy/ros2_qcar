import numpy as np
import time
import os 
from qvl.qlabs import QuanserInteractiveLabs
from Controller.idm_control import IDMControl
from Controller.CACC import CACC
from md_vehicle import Vehicle
from md_environment import Environment
from md_logging_config import logger


def main():
    # Clear terminal (cross-platform alternative)
    os.system('clear' if os.name == 'posix' else 'cls')
    qlabs = QuanserInteractiveLabs()
    print("Connecting to QLabs...")
    try:
        qlabs.open("localhost")
        print("Connected to QLabs")
    except:
        print("Unable to connect to QLabs")
        quit()

    # Setup environment
    env = Environment(qlabs)
    env.setup()
    leader_qcar, follower_qcar = env.spawn_vehicles()
    env.start_real_time()
    time.sleep(10)  # Allow real-time model to stabilize

    # Initialize controller
    class DummyController:
        def __init__(self):
            self.param_opt = {
                'alpha': 1.0,
                'beta': 1.5,
                'v0': 1.0,
                'delta': 4,
                'T': 0.4,
                's0': 1,
                'ri': 0.5,
                'hi': 0.5,
                'K': np.array([[1, 0.0], [0.0, 1]])
            }
            self.param_sys = None
            self.goal = None
            self.straightlane = None
            self.vehicle_number = 1

        def get_surrounding_vehicles(self, *args, **kwargs):
            return None, [None], None, None

    controller = DummyController()
    control_algo = CACC(controller) if True else IDMControl(controller)

    # Create vehicles
    leader_vehicle = Vehicle(qcar=leader_qcar, idm_controller=control_algo, vehicle_id=0, is_leader=True, send_port=6001, recv_port=6000, ack_port=6002)
    follower_vehicle = Vehicle(qcar=follower_qcar, idm_controller=control_algo, vehicle_id=1, is_leader=False, send_port=6000, recv_port=6001, ack_port=6003)

    # Start vehicles
    logger.info("Starting simulation")  # Uses default vehicle_id 'N/A' from formatter
    leader_vehicle.start()
    follower_vehicle.start()

    # Run simulation for 50 seconds
    time.sleep(50)

    # Stop vehicles and clean up
    leader_vehicle.stop()
    follower_vehicle.stop()
    env.cleanup()
    logger.info("Simulation ended.")

if __name__ == "__main__":
    main()