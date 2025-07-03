
import numpy as np
import time

from qvl.qlabs import QuanserInteractiveLabs
from qvl.qcar2 import QLabsQCar2
from qvl.real_time import QLabsRealTime
from qvl.basic_shape import QLabsBasicShape
from qvl.walls import QLabsWalls
from qvl.qcar_flooring import QLabsQCarFlooring
from qvl.crosswalk import QLabsCrosswalk
from Controller.idm_control import IDMControl
from Controller.CACC import CACC
import os
from Vehicle4 import Vehicle

def main():
    # Initialize QLabs
    os.system('cls')
    qlabs = QuanserInteractiveLabs()
    print("Connecting to QLabs...")
    try:
        qlabs.open("localhost")
        print("Connected to QLabs")
    except:
        print("Unable to connect to QLabs")
        quit()

    qlabs.destroy_all_spawned_actors()
    QLabsRealTime().terminate_all_real_time_models()

    # Setup environment
    x_offset = 0.13
    y_offset = 1.67
    hFloor = QLabsQCarFlooring(qlabs)
    hFloor.spawn_degrees([x_offset, y_offset, 0.001], rotation=[0, 0, -90], configuration=0)
    hWall = QLabsWalls(qlabs)
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
    myCrossWalk = QLabsCrosswalk(qlabs)
    myCrossWalk.spawn_degrees(location=[-2 + x_offset, -1.475 + y_offset, 0.01], rotation=[0, 0, 0], scale=[0.1, 0.1, 0.075], configuration=0)
    mySpline = QLabsBasicShape(qlabs)
    mySpline.spawn_degrees(location=[2.05 + x_offset, -1.5 + y_offset, 0.01], rotation=[0, 0, 0], scale=[0.27, 0.02, 0.001], waitForConfirmation=False)

    # Initialize vehicles
    leader = QLabsQCar2(qlabs)
    follower = QLabsQCar2(qlabs)
    leader.spawn_id(actorNumber=0, location=[-1.205, -0.83, 0.005], rotation=[0, 0, -44.7], scale=[0.1, 0.1, 0.1])
    follower.spawn_id(actorNumber=1, location=[-1.735, -0.35, 0.005], rotation=[0, 0, -44.7], scale=[0.1, 0.1, 0.1])

    rtModel = os.path.normpath(os.path.join(os.environ['RTMODELS_DIR'], 'QCar2/QCar2_Workspace_studio'))
    QLabsRealTime().start_real_time_model(rtModel, actorNumber=0)

    time.sleep(10)

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
    leader_vehicle = Vehicle(qcar=leader, idm_controller=control_algo, vehicle_id=0, is_leader=True, send_port=6001, recv_port=6000,ack_port=6002)
    follower_vehicle = Vehicle(qcar=follower, idm_controller=control_algo, vehicle_id=1, is_leader=False, send_port=6000, recv_port=6001,ack_port=6003)

    # Start vehicles
    leader_vehicle.start()
    follower_vehicle.start()

    # Run simulation for 50 seconds
    time.sleep(50)

    # Stop vehicles
    leader_vehicle.stop()
    follower_vehicle.stop()

    qlabs.close()
    print("Simulation ended.")

if __name__ == "__main__":
    main()