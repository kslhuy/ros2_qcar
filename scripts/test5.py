# convoi de 3 voitures
from follower5 import Follower
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

USE_CACC = True  # Set to False to use IDM instead
# USE_CACC = False

class DummyController:
    def __init__(self,qcar_id):
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
        self.vehicle_id = qcar_id
        self.vehicle_number = qcar_id

    def get_surrounding_vehicles(self, *args, **kwargs):
        return None, [dummy_leader], None, None  # You override this anyway



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
second_follower = QLabsQCar2(qlabs)

leader_id = 0
follower_id = 1
second_follower_id = 2

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
second_follower.spawn_id(actorNumber=second_follower_id, location=[-2.162, 0.023, 0.005], 
                         rotation=[0, 0, -44.7], scale=[0.1, 0.1, 0.1])

rtModel = os.path.normpath(os.path.join(os.environ['RTMODELS_DIR'], 'QCar2/QCar2_Workspace_studio'))
# QLabsRealTime().start_real_time_model(rtModel, actorNumber=0)  
QLabsRealTime().start_real_time_model(rtModel, actorNumber=leader_id)
# QLabsRealTime().start_real_time_model(rtModel, actorNumber=follower_id)
# QLabsRealTime().start_real_time_model(rtModel, actorNumber=second_follower_id)

# Wait for stability
time.sleep(3)

## Choose Controller ##
# controller = DummyController()
controller_follower1 = DummyController(follower_id)
controller_follower2 = DummyController(second_follower_id)

# if USE_CACC:
#     control_algo = CACC(controller)
# else:
#     control_algo = IDMControl(controller)
if USE_CACC:
    control_algo1 = CACC(controller_follower1)
    control_algo2 = CACC(controller_follower2)
else:
    control_algo1 = IDMControl(controller_follower1)
    control_algo2 = IDMControl(controller_follower2)

f = Follower(qcar=follower, idm_controller=control_algo1, vehicle_id=follower_id)
f2 = Follower(qcar=second_follower, idm_controller=control_algo2, vehicle_id=second_follower_id)


f.running = True
f2.running = True

print("Using CACC" if USE_CACC else "Using IDM")


while f.running and f2.running == True:
    f.follow(leader)
    f2.follow(follower)
    time.sleep(0.05)
    
# def run_follower_safely(follower, leader):
#     try:
#         follower.run(leader)
#     except Exception as e:
#         print(f"[Follower-{follower.vehicle_id}] Error: {str(e)}")
#     finally:
#         print(f"[Follower-{follower.vehicle_id}] Thread exiting")

# print("Using CACC" if USE_CACC else "Using IDM")

# # # f = Follower(qcar=follower, idm_controller=idm)
# # f.run(leader=leader)
# # f2.run(leader=follower)

# print(f"Follower actorNumber: {follower.actorNumber}")


# import threading

# # # Manual threads, calling run() yourself
# # thread1 = threading.Thread(target=f.run, args=(leader,))
# # thread2 = threading.Thread(target=f2.run, args=(follower,))

# # thread1.start()
# # thread2.start()

# # Create and start threads with proper error handling
# thread1 = threading.Thread(target=run_follower_safely, args=(f, leader))
# thread2 = threading.Thread(target=run_follower_safely, args=(f2, follower))

# thread1.daemon = True  # Mark as daemon threads so they exit when main thread exits
# thread2.daemon = True

# thread1.start()
# thread2.start()


# # # Cleanup
# # qlabs.close()

# try:
#     # Main thread can do other work or just wait
#     while True:
#         time.sleep(1)
# except KeyboardInterrupt:
#     print("Shutting down...")
# finally:
#     # Cleanup
#     qlabs.close()
#     print("QLabs connection closed")
