from trustFollower3 import Follower
from qvl.qlabs import QuanserInteractiveLabs
from qvl.qcar2 import QLabsQCar2
from qvl.real_time import QLabsRealTime
import time, os
import numpy as np
from qvl.basic_shape import QLabsBasicShape
from qvl.walls import QLabsWalls
from qvl.qcar_flooring import QLabsQCarFlooring
from qvl.crosswalk import QLabsCrosswalk
from Controller.idm_control import IDMControl
from Controller.CACC import CACC

USE_CACC = True

class DummyController:
    def __init__(self, qcar_id):
        self.param_opt = {...}  # same as before
        self.vehicle_id = qcar_id
        self.vehicle_number = qcar_id
    def get_surrounding_vehicles(self, *args, **kwargs):
        return None, [], None, None

os.system('cls')
qlabs = QuanserInteractiveLabs()
qlabs.open("localhost")
qlabs.destroy_all_spawned_actors()
QLabsRealTime().terminate_all_real_time_models()

leader = QLabsQCar2(qlabs)
follower = QLabsQCar2(qlabs)
second_follower = QLabsQCar2(qlabs)

leader.spawn_id(0, [-1.205, -0.83, 0.005], [0, 0, -44.7], [0.1, 0.1, 0.1])
follower.spawn_id(1, [-1.735, -0.35, 0.005], [0, 0, -44.7], [0.1, 0.1, 0.1])
second_follower.spawn_id(2, [-2.162, 0.023, 0.005], [0, 0, -44.7], [0.1, 0.1, 0.1])

rtModel = os.path.normpath(os.path.join(os.environ['RTMODELS_DIR'], 'QCar2/QCar2_Workspace_studio'))
QLabsRealTime().start_real_time_model(rtModel, 0)

controller1 = DummyController(1)
controller2 = DummyController(2)

control_algo1 = CACC(controller1) if USE_CACC else IDMControl(controller1)
control_algo2 = CACC(controller2) if USE_CACC else IDMControl(controller2)

f = Follower(follower, control_algo1, 1)
f2 = Follower(second_follower, control_algo2, 2)

f.running = True
f2.running = True

while f.running and f2.running:
    f.follow(leader)
    f2.follow(follower)
    time.sleep(0.05)

qlabs.close()
