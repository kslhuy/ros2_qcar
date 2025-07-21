import threading
from follower import Follower
from qvl.qcar_flooring import QLabsQCarFlooring
from qvl.qlabs import QuanserInteractiveLabs
from qvl.qcar2 import QLabsQCar2
from qcar2_sim import QCar2Sim
from qvl.real_time import QLabsRealTime
import os
from qvl.walls import QLabsWalls


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
follower2 = QLabsQCar2(qlabs)
follower3 = QLabsQCar2(qlabs)

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


# Spawn cars
initialPosition = [-1.205, -0.83, 0.005]
initialOrientation = [0, 0, -44.7]
# leader = QLabsQCar2(qlabs)
# leader.spawn_id(actorNumber=leader_id, 
#             location=initialPosition, 
#             rotation=initialOrientation,
#             scale=[.1, .1, .1], 
#             configuration=0, 
#             waitForConfirmation=True)

# rtModel = os.path.normpath(os.path.join(os.environ['RTMODELS_DIR'], 'QCar2/QCar2_Workspace_studio_interleaved'))
# QLabsRealTime().start_real_time_model(rtModel, leader_id)

lock = threading.Lock()


follower.spawn_id(
    actorNumber=follower_id, 
    location=initialPosition, 
    rotation=initialOrientation,
    scale=[0.1, 0.1, 0.1]
)

car = QCar2Sim(follower, follower_id, lock=lock)
threading.Thread(target=car.start).start()


follower2.spawn_id(
    actorNumber=2, 
    location=[0, 0, 0], 
    rotation=[0, 0, 0],
    scale=[0.1, 0.1, 0.1]
)

car2 = QCar2Sim(follower2, 2, lock=lock)
threading.Thread(target=car2.start).start()


follower3.spawn_id(
    actorNumber=3, 
    location=[1, 0, 0], 
    rotation=[0, 0, 0],
    scale=[0.1, 0.1, 0.1]
)

car3 = QCar2Sim(follower3, 3, lock=lock)
threading.Thread(target=car3.start).start()

# Cleanup
# qlabs.close()
