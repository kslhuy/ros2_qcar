from follower import Follower
from qvl.qlabs import QuanserInteractiveLabs
from qvl.qcar2 import QLabsQCar2
from qvl.real_time import QLabsRealTime
import os


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

# Spawn cars
leader.spawn_id(
    actorNumber=leader_id, 
    location=[0, 0, 0], 
    rotation=[0, 0, 0],
    scale=[0.1, 0.1, 0.1]
)
follower.spawn_id(
    actorNumber=follower_id, 
    location=[-5.5, -5, 0], 
    rotation=[0, 0, 0], 
    scale=[0.1, 0.1, 0.1]
)

rtModel = os.path.normpath(os.path.join(os.environ['RTMODELS_DIR'], 'QCar2/QCar2_Workspace_studio'))
QLabsRealTime().start_real_time_model(rtModel, actorNumber=0)  

f = Follower(qcar=follower)
f.run(leader=leader)

# Cleanup
qlabs.close()
