from observerFollower import Follower
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

from Observer.observer import Observer

USE_CACC = True  # Set to False to use IDM instead
# USE_CACC = False

class DummyController:
    def __init__(self, vehicle=None):
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
        self.vehicle = vehicle

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
# QLabsRealTime().start_real_time_model(rtModel, actorNumber=0)
QLabsRealTime().start_real_time_model(rtModel, actorNumber=leader_id)
# QLabsRealTime().start_real_time_model(rtModel, actorNumber=follower_id)  


# Vehicle state structure assumption
class VehicleWrapper:
    def __init__(self, qcar, vehicle_number):
        self.qcar = qcar
        self.vehicle_number = vehicle_number
        self.input = np.array([0.0, 0.0])
        self.state = np.array([0.0, 0.0, 0.0, 0.0])
        self.other_vehicles = []
        self.total_time_step = 1000
        self.state_log = np.zeros((4, 1000))
        self.scenarios_config = {
            "Local_observer_type": "kalman",
            "Is_noise_mesurement": False,
            "Use_predict_observer": False,
            "predict_controller_type": "true_other"
        }
        self.center_communication = None  # Implement communication stub if needed
        self.trip_models = []

# Simulate basic setup
veh_param = {"dt": 0.1}
initial_global = np.zeros((4, 2))  # For 2 vehicles
initial_local = np.zeros(4)

# Wrap follower as the observing vehicle
obs_vehicle = VehicleWrapper(follower, vehicle_number=1)
# obs_vehicle.other_vehicles = [None, obs_vehicle]  # Dummy 2nd vehicle for indexing
leader_wrapper = VehicleWrapper(leader, vehicle_number=0)
obs_vehicle.other_vehicles = [leader_wrapper, obs_vehicle]

## Choose Controller ##
controller = DummyController(vehicle=obs_vehicle)


if USE_CACC:
    control_algo = CACC(controller)
else:
    control_algo = IDMControl(controller)


# Define and assign dummy trip models (needed by observer)
class DummyTripModel:
    def __init__(self, flag=True):
        self.flag_local_est_check = flag

obs_vehicle.trip_models = [DummyTripModel(), DummyTripModel()]

# class DummyCommunication:
#     def get_local_state(self, j, host_id):
#         # Always return zeros (4-element state)
#         return np.zeros(4)

#     def get_global_state(self, k, host_id):
#         # Return a dummy 4xN state matrix (e.g., 2 vehicles)
#         return np.zeros((4, 2))
class DummyCommunication:
    def get_global_state(self, k, host_id):
        # Return real-time leader state when host is the follower
        if k == 0:  # assuming leader = vehicle 0
            _, pos, rot, _ = leader.get_world_transform()
            state = np.array([pos[0], pos[1], rot[2], 0.5])
            return np.column_stack([state, np.zeros(4)])  # [leader, follower]
        return np.zeros((4, 2))
    
    def get_local_state(self, j, host_id):
        # Always return zeros (4-element state)
        return np.zeros(4)


obs_vehicle.center_communication = DummyCommunication()


observer = Observer(vehicle=obs_vehicle, veh_param=veh_param,
                    initial_global_state=initial_global,
                    initial_local_state=initial_local)

obs_vehicle.observer = observer

f = Follower(qcar=follower, idm_controller=control_algo, vehicle_id=1)
f.set_observer(observer)

print("Using CACC" if USE_CACC else "Using IDM")

try:
    f.run(leader=leader)
finally:
    # Save logs on exit (even from Ctrl+C)
    true_states = np.array(f.true_state_log)
    est_states = np.array(f.observer.est_global_state_log[:, :len(true_states), 0]).T
    time_series = np.array(f.time_log)
    time_series -= time_series[0]

    os.makedirs("logs", exist_ok=True)
    np.savez("logs/observer_validation.npz", time=time_series, true=true_states, estimated=est_states)
    print("[INFO] Observer log saved to logs/observer_validation.npz")

    qlabs.close()


# Cleanup
qlabs.close()
