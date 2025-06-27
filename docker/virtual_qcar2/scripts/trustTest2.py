# trustTest2.py
from trustFollower2 import Follower
from qvl.qlabs import QuanserInteractiveLabs
from qvl.qcar2 import QLabsQCar2
from qvl.real_time import QLabsRealTime
import time, os, math
import numpy as np
from Trust.TriPTrustModel2 import TriPTrustModel

from qvl.basic_shape import QLabsBasicShape
from qvl.walls import QLabsWalls
from qvl.qcar_flooring import QLabsQCarFlooring
from qvl.crosswalk import QLabsCrosswalk

from Controller.idm_control import IDMControl
from Controller.CACC import CACC

from communication import Comm


USE_CACC = True

# class Comm:
#     def __init__(self, controller,leader_state):
#         self.controller = controller
#         self.controller.leader_state = leader_state

#     # def get_local_state(self, id, host_id):
#     #     return np.zeros(4) if id != host_id else self.controller.state
#     def get_local_state(self, id, host_id):
#         if id == self.controller.vehicle_number:
#             return self.controller.state
#         elif hasattr(self.controller, 'leader_state'):
#             return self.controller.leader_state
#         return np.zeros(4)

#     def get_global_state(self, id, host_id):
#         return self.controller.observer.est_global_state_current
#     def get_input(self, id):
#         return np.zeros(1)
# class Comm:
#     def __init__(self, host_controller, vehicle_dict):
#         self.controller = host_controller
#         self.vehicles = vehicle_dict  # {vehicle_id: controller}

#     def get_local_state(self, id, host_id):
#         if id in self.vehicles:
#             return self.vehicles[id].state
#         return np.zeros(4)

#     def get_global_state(self, id, host_id):
#         return self.controller.observer.est_global_state_current

#     def get_input(self, id):
#         if id in self.vehicles:
#             return self.vehicles[id].input
#         return np.zeros(1)


class DummyController:
    def __init__(self, qcar_id):
        self.param_opt = {
            'alpha': 1.0, 'beta': 1.5, 'v0': 1.0, 'delta': 4, 'T': 0.4, 's0': 1,
            'ri': 0.5, 'hi': 0.5, 'K': np.array([[1, 0.0], [0.0, 1]])
        }
        self.param_sys = {'l_r': 0.128, 'dt': 0.05}
        self.param = type('Param', (), {'l_r': self.param_sys['l_r']})()
        self.dt = 0.05
        self.goal = None
        self.straightlane = None
        self.vehicle_id = qcar_id
        self.vehicle_number = qcar_id
        self.state = np.zeros(4)
        self.input = np.zeros(1)
        self.scenarios_config = type('Config', (), {
            'Monitor_sudden_change': True, 'Dichiret_type': 'Dual'
        })()
        self.observer = type('Observer', (), {
            'est_local_state_current': np.zeros(4),
            'est_global_state_current': np.zeros((4, 3))
        })()
        # self.center_communication = Comm(self)

    def get_surrounding_vehicles(self, *args, **kwargs):
        return None, [dummy_leader], None, None

    def update_state(self, pos, rot, v, a):
        self.state = np.array([pos[0], pos[1], rot[2], v])
        self.observer.est_local_state_current = self.state
        self.observer.est_global_state_current[:, self.vehicle_number] = self.state
        self.input[0] = a

# Connect to QLabs
os.system('clear')
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

leader = QLabsQCar2(qlabs)
follower = QLabsQCar2(qlabs)
second_follower = QLabsQCar2(qlabs)

leader_id = 0
follower_id = 1
second_follower_id = 2

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

leader.spawn_id(actorNumber=leader_id, location=[-1.205, -0.83, 0.005], rotation=[0, 0, -44.7], scale=[0.1, 0.1, 0.1])
follower.spawn_id(actorNumber=follower_id, location=[-1.735, -0.35, 0.005], rotation=[0, 0, -44.7], scale=[0.1, 0.1, 0.1])
second_follower.spawn_id(actorNumber=second_follower_id, location=[-2.162, 0.023, 0.005], rotation=[0, 0, -44.7], scale=[0.1, 0.1, 0.1])

rtModel = os.path.normpath(os.path.join(os.environ['RTMODELS_DIR'], 'QCar2/QCar2_Workspace_studio'))
QLabsRealTime().start_real_time_model(rtModel, actorNumber=leader_id)

time.sleep(3)

controller_leader = DummyController(leader_id)
controller_follower1 = DummyController(follower_id)
controller_follower2 = DummyController(second_follower_id)

vehicle_dict = {
    0: controller_leader,
    1: controller_follower1,
    2: controller_follower2
}

controller_leader.center_communication = Comm(controller_leader, vehicle_dict)
controller_follower1.center_communication = Comm(controller_follower1, vehicle_dict)
controller_follower2.center_communication = Comm(controller_follower2, vehicle_dict)


trust_model1 = TriPTrustModel()
trust_model2 = TriPTrustModel()

if USE_CACC:
    control_algo1 = CACC(controller_follower1)
    control_algo2 = CACC(controller_follower2)
else:
    control_algo1 = IDMControl(controller_follower1)
    control_algo2 = IDMControl(controller_follower2)

f = Follower(qcar=follower, idm_controller=control_algo1, vehicle_id=follower_id, trust_model=trust_model1)
f2 = Follower(qcar=second_follower, idm_controller=control_algo2, vehicle_id=second_follower_id, trust_model=trust_model2)

f.running = True
f2.running = True

print("Using CACC" if USE_CACC else "Using IDM")

prev_pos_leader = None  # Define before the loop

try:
    while f.running and f2.running:
        _, pos_leader, rot_leader, _ = leader.get_world_transform()

        # Estimate velocity
        if prev_pos_leader is None:
            v_leader = 0.0
        else:
            dx = pos_leader[0] - prev_pos_leader[0]
            dy = pos_leader[1] - prev_pos_leader[1]
            print('dx',math.hypot(dx, dy))
            dt = 0.2
            v_leader = math.hypot(dx, dy) / dt

        prev_pos_leader = pos_leader

        controller_leader.update_state(pos_leader, rot_leader, v_leader, 0)
        f.follow(leader)
        f2.follow(follower)
        time.sleep(0.05)

except KeyboardInterrupt:
    print("Shutting down...")
finally:
    f.stop()
    f2.stop()
    qlabs.close()
    print("QLabs connection closed")