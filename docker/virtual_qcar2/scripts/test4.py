####### Communication unidirectionnel #########

from follower4 import Follower
from qvl.qlabs import QuanserInteractiveLabs
from qvl.qcar2 import QLabsQCar2
from qvl.real_time import QLabsRealTime
import time, os, math
import numpy as np

from qvl.basic_shape import QLabsBasicShape
from qvl.walls import QLabsWalls
from qvl.qcar_flooring import QLabsQCarFlooring
from qvl.crosswalk import QLabsCrosswalk

from Controller.idm_control import IDMControl
from Controller.CACC import CACC


import socket
import pickle
import threading

# def broadcast_leader_pose():
#     sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     follower_ip = '127.0.0.1'
#     port = 5005
#     while True:
#         _, pos_leader, rot_leader, _ = leader.get_world_transform()
#         v_leader = leader.motorTach if hasattr(leader, 'motorTach') else 0.5
#         data = {'pos': pos_leader, 'rot': rot_leader, 'v': v_leader}
#         sock.sendto(pickle.dumps(data), (follower_ip, port))
#         # print("sent")
#         time.sleep(0.1)

def broadcast_leader_pose():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    follower_ip = '127.0.0.1'
    port = 5005
    sequence_number = 0  # Add sequence counter

    while True:
        _, pos_leader, rot_leader, _ = leader.get_world_transform()
        v_leader = 0.5
        data = {
            'seq': sequence_number,
            'pos': pos_leader,
            'rot': rot_leader,
            'v': v_leader
        }
        sock.sendto(pickle.dumps(data), (follower_ip, port))
        sequence_number += 1
        time.sleep(0.1)





USE_CACC = True  # Set to False to use IDM instead
# USE_CACC = False


class DummyController:
    def __init__(self):
        self.param_opt = {
            'alpha': 1.0,
            'beta': 1.5,
            'v0': 1.0,
            'delta': 4,
            'T': 0.4,
            's0': 1,  # for IDM
            'ri': 0.5,  # for CACC
            'hi': 0.5,
            'K': np.array([[1, 0.0], [0.0, 1]])  # for CACC
        }
        self.param_sys = None
        self.goal = None
        self.straightlane = None
        self.vehicle_number = 1

    def get_surrounding_vehicles(self, *args, **kwargs):
        return None, [dummy_leader], None, None  # override this anyway





# Connect to QLabs
os.system('cls')
qlabs = QuanserInteractiveLabs()
print("Connecting to QLabs...")
try:
    qlabs.open("localhost")
    print("Connected to QLabs")
except:
    print("Unable to connect to QLabs")
    quit()
    
# Delete any previous QCar instances and stop any running spawn models
qlabs.destroy_all_spawned_actors()
QLabsRealTime().terminate_all_real_time_models()


# Create leader and follower QCars
leader = QLabsQCar2(qlabs)
follower = QLabsQCar2(qlabs)


threading.Thread(target=broadcast_leader_pose, daemon=True).start()

leader_id = 0
follower_id = 1

x_offset = 0.13
y_offset = 1.67
hFloor = QLabsQCarFlooring(qlabs)
hFloor.spawn_degrees([x_offset, y_offset, 0.001],rotation = [0, 0, -90], configuration=0)


### region: Walls
hWall = QLabsWalls(qlabs)
hWall.set_enable_dynamics(False)

for y in range (5):
    hWall.spawn_degrees(location=[-2.4 + x_offset, (-y*1.0)+2.55 + y_offset, 0.001], rotation=[0, 0, 0])

for x in range (5):
    hWall.spawn_degrees(location=[-1.9+x + x_offset, 3.05+ y_offset, 0.001], rotation=[0, 0, 90])

for y in range (6):
    hWall.spawn_degrees(location=[2.4+ x_offset, (-y*1.0)+2.55 + y_offset, 0.001], rotation=[0, 0, 0])

for x in range (4):
    hWall.spawn_degrees(location=[-0.9+x+ x_offset, -3.05+ y_offset, 0.001], rotation=[0, 0, 90])

hWall.spawn_degrees(location=[-2.03 + x_offset, -2.275+ y_offset, 0.001], rotation=[0, 0, 48])
hWall.spawn_degrees(location=[-1.575+ x_offset, -2.7+ y_offset, 0.001], rotation=[0, 0, 48])


# Spawning crosswalks
myCrossWalk = QLabsCrosswalk(qlabs)
myCrossWalk.spawn_degrees   (location =[-2 + x_offset, -1.475 + y_offset, 0.01],
                            rotation=[0,0,0], 
                            scale = [0.1,0.1,0.075],
                            configuration = 0)

mySpline = QLabsBasicShape(qlabs)
mySpline.spawn_degrees (location=[2.05 + x_offset, -1.5 + y_offset, 0.01], 
                        rotation=[0, 0, 0], 
                        scale=[0.27, 0.02, 0.001], 
                        waitForConfirmation=False)

# Spawn cars
leader.spawn_id(actorNumber=leader_id, location=[-1.205, -0.83, 0.005], rotation=[0, 0, -44.7],
        scale=[0.1, 0.1, 0.1])
follower.spawn_id(actorNumber=follower_id, location=[-1.735, -0.35, 0.005], 
        rotation=[0, 0, -44.7], scale=[0.1, 0.1, 0.1])

rtModel = os.path.normpath(os.path.join(os.environ['RTMODELS_DIR'], 'QCar2/QCar2_Workspace_studio'))
QLabsRealTime().start_real_time_model(rtModel, actorNumber=0)  


## Choose Controller ##
controller = DummyController()

if USE_CACC:
    control_algo = CACC(controller)
else:
    control_algo = IDMControl(controller)

f = Follower(qcar=follower, idm_controller=control_algo, vehicle_id=1)

print("Using CACC" if USE_CACC else "Using IDM")

# f = Follower(qcar=follower, idm_controller=idm)
f.run()


# Cleanup
qlabs.close()
