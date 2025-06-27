###### look ahead controller ######
from follower3 import Follower
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
from Controller.look_ahead_Control import LookAheadControl

# USE_CACC = True  # Set to False to use IDM instead
# USE_CACC = False

# # Dummy controller with param_opt stub
# class DummyController:
#     def __init__(self):
#         self.param_opt = {
#             'alpha': 1.5,  # max acceleration
#             'beta': 1.5,   # comfortable deceleration
#             'v0': 1.0,     # desired speed
#             'delta': 4,    # acceleration exponent
#             'T': 0.5,      # safe time headway
#             's0': 0.1      # minimum gap
#         }
#         self.param_sys = None
#         self.goal = None
#         self.straightlane = None
#         self.vehicle_number = 1

#     def get_surrounding_vehicles(self, *args, **kwargs):
#         return None, [], None, None

# idm = IDMControl(DummyController())


class DummyController:
    def __init__(self):
        self.param_opt = {
            # IDM parameters
            'alpha': 1.0,
            'beta': 1.5,
            'v0': 1.0,
            'delta': 4,
            'T': 0.4,
            's0': 1.0,

            # CACC parameters
            'ri': 0.4, # desired standstill distance
            'hi': 0.5, # time-gap
            'K': np.array([[1.0, 0.0], [0.0, 1.0]]),

            # Look-ahead shares ri and hi
        }

        self.param_sys = {
            'l_r': 0.128, # the distance between the rear tires and the center of gravity
            'l_f': 0.128, # the distance between the front tires and the center of gravity
            'C1': 0.5, 
            'C2': 0.5,
            'mass': 2.0
        }

        self.goal = None
        self.straightlane = None
        self.vehicle_number = 1

    def get_surrounding_vehicles(self, *args, **kwargs):
        return None, [dummy_leader], None, None

CONTROLLER_TYPE = 'lookahead'  # or 'idm', 'cacc','lookahead'


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

# if USE_CACC:
#     control_algo = CACC(controller)
# else:
#     control_algo = IDMControl(controller)
# f = Follower(qcar=follower, idm_controller=control_algo, vehicle_id=1)

idm_controller = IDMControl(controller)
cacc_controller = CACC(controller)
lookahead_controller = LookAheadControl(controller)



if CONTROLLER_TYPE == 'idm':
    selected_controller = idm_controller
elif CONTROLLER_TYPE == 'cacc':
    selected_controller = cacc_controller
elif CONTROLLER_TYPE == 'lookahead':
    selected_controller = lookahead_controller
else:
    raise ValueError("Unknown controller type")

f = Follower(qcar=follower, controller=selected_controller, vehicle_id=1)
f.run(leader=leader)


# print(f"Using controller: {CONTROLLER_TYPE.upper()}")


# print("Using CACC" if USE_CACC else "Using IDM")


# f.run(leader=leader)

# Cleanup
qlabs.close()
