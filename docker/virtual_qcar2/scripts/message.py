import numpy as np
import time
import os
from qvl.qlabs import QuanserInteractiveLabs
from qvl.qcar2 import QLabsQCar2
from qvl.real_time import QLabsRealTime
from Controller.idm_control import IDMControl
from Controller.CACC import CACC
# from Vehicle2 import Vehicle

def main():
    # Initialize QLabs connection
    os.system('cls')
    qlabs = QuanserInteractiveLabs()
    print("Connecting to QLabs...")
    server_ip = "192.168.137.1"  # Replace with the server PC's IP address
    if not qlabs.open(server_ip):
        print("Unable to connect to QLabs")
        return

    print("Connected to QLabs")

    # Do not destroy all actors
    # Initialize follower vehicle
    follower = QLabsQCar2(qlabs)
    follower.spawn_id(actorNumber=1, location=[-2, -0.35, 0.005], rotation=[0, 0, -44.7], scale=[0.1, 0.1, 0.1])

    # # Start real-time model
    # rtModel = os.path.normpath(os.path.join(os.environ['RTMODELS_DIR'], 'QCar2/QCar2_Workspace_studio'))
    # QLabsRealTime().start_real_time_model(rtModel, actorNumber=1)

    

    # # Initialize controller
    # class DummyController:
    #     def __init__(self):
    #         self.param_opt = {
    #             'alpha': 1.0, 'beta': 1.5, 'v0': 1.0, 'delta': 4, 'T': 0.4, 's0': 1, 'ri': 0.5, 'hi': 0.5,
    #             'K': np.array([[1, 0.0], [0.0, 1]])
    #         }
    #         self.param_sys = None
    #         self.goal = None
    #         self.straightlane = None
    #         self.vehicle_number = 1

    #     def get_surrounding_vehicles(self, *args, **kwargs):
    #         return None, [None], None, None

    # controller = DummyController()
    # control_algo = CACC(controller) if True else IDMControl(controller)

    # # Create and start follower vehicle
    # follower_vehicle = Vehicle(qcar=follower, idm_controller=control_algo, vehicle_id=1, is_leader=False, send_port=5050, recv_port=5051)
    # follower_vehicle.start()

    # # Run simulation
    # time.sleep(30)
    
    follower.set_velocity_and_request_state(
        forward=0.5,
        turn=0,
        headlights=False,
        leftTurnSignal=False,
        rightTurnSignal=False,
        brakeSignal=False,
        reverseSignal=False
    )


    time.sleep(5000)
    # Cleanup
    # follower_vehicle.stop()
    qlabs.close()
    print("Simulation ended.")

if __name__ == "__main__":
    main()