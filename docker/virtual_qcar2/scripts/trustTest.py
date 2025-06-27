from trustFollower import Follower
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

import socket, json, threading

import pickle


# def broadcast_leader_pose():
#     sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

#     follower_ip = '127.0.0.1'  # Replace with actual IP if follower is remote
#     port = 9999
#     sequence_number = 0

#     while True:
#         _, pos, rot, _ = leader.get_world_transform()
#         v_leader = 0.5  # Replace if needed

#         data = {
#             'seq': sequence_number,
#             'x': pos[0],
#             'y': pos[1],
#             'theta': rot[2],
#             'v': v_leader,
#             'a': 0.0
#         }

#         sock.sendto(pickle.dumps(data), (follower_ip, port))
#         sequence_number += 1
#         time.sleep(0.2)

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

def leader_udp_broadcast_loop():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = ("localhost", 9999)  # Follower listener port

    start_time = time.time()
    while True:
        _, pos, rot, _ = leader.get_world_transform()
        data = {
            "x": pos[0],
            "y": pos[1],
            "theta": rot[2],
            "v": 1.0,
            "a": 0.0
        }

        # Inject spoof after 10 sec
        if time.time() - start_time > 10:
            data["x"] += 2.0  # Simulate GPS spoof

        sock.sendto(json.dumps(data).encode(), dest)
        print("[Leader] Sent: ping")
        time.sleep(0.2)


# def leader_udp_broadcast_loop():
#     sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     dest = ("localhost", 9999)
#     while True:
#         sock.sendto(b"ping", dest)  # send a simple word
#         print("[Leader] Sent: ping")
#         time.sleep(1.0)



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

threading.Thread(target=leader_udp_broadcast_loop, daemon=True).start()

# Wait a moment to let model fully initialize
time.sleep(0.5)
## Choose Controller ##
# controller = DummyController()

controller = DummyController(follower_id)

if USE_CACC:
    control_algo = CACC(controller)
else:
    control_algo = IDMControl(controller)

f = Follower(qcar=follower, idm_controller=control_algo, vehicle_id=1)
f.running = True
print("Using CACC" if USE_CACC else "Using IDM")

try:
    while f.running:
        msg = f.udp_listener.get_latest()
        if msg:
            leader_state = {
                'position': [msg.get('x', 0.0), msg.get('y', 0.0), 0.005],
                'rotation': [0, 0, msg.get('theta', 0.0)],
                'velocity': msg.get('v', 0.0),
                'acceleration': msg.get('a', 0.0)
            }

            f.control.update_leader_state(leader_state)
            print("[Follower] Received:", leader_state)

            # Log trust score and decision factors
            trust_score = f.trust_model.calculate_trust_score(f.trust_model.rating_vector)
            gamma = f.trust_model.gamma_cross_log[-1] if f.trust_model.gamma_cross_log else 0
            d_score = f.trust_model.d_score_log[-1] if f.trust_model.d_score_log else 0
            v_score = f.trust_model.v_score_log[-1] if f.trust_model.v_score_log else 0

            f.trust_log.append([time.time(), trust_score, gamma, v_score, d_score])

        else:
            print("[Follower] Waiting...")

        if f.control.leader_state is not None:
            pos_leader = f.control.leader_state['position']
            rot_leader = f.control.leader_state['rotation']

            _, pos_follower, rot_follower, _ = follower.get_world_transform()

            # Project leader forward
            lookahead_distance = 0.4
            target_x = pos_leader[0] - lookahead_distance * math.cos(rot_leader[2])
            target_y = pos_leader[1] - lookahead_distance * math.sin(rot_leader[2])

            dx = target_x - pos_follower[0]
            dy = target_y - pos_follower[1]
            distance = math.hypot(dx, dy)

            follower_heading = rot_follower[2]
            target_angle = math.atan2(dy, dx)
            heading_error = (target_angle + math.pi) % (2 * math.pi) - math.pi - follower_heading
            heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

            # Steering
            steering_cmd = -f.k_steering * heading_error
            steering_cmd = max(-f.max_steering, min(f.max_steering, steering_cmd))

            # Velocities
            v_follower = getattr(f.qcar, 'motorTach', 0.5)
            v_leader = f.control.leader_state.get('velocity', 0.5)
            a_leader = f.control.leader_state.get('acceleration', 0.0)
            b_leader = 0.2

            # Trust eval
            v_score = f.trust_model.evaluate_velocity(
                host_id=1, target_id=0,
                v_y=v_follower, v_host=v_follower,
                v_leader=v_leader, a_leader=a_leader, b_leader=b_leader
            )
            f.trust_model.v_score_log.append(v_score)

            d_score = f.trust_model.evaluate_distance(distance, f.default_safe_distance)
            f.trust_model.d_score_log.append(d_score)

            gamma_cross = abs(v_follower - v_leader)
            D_pos = distance
            D_vel = abs(v_follower - v_leader)

            f.trust_model.gamma_cross_log.append(gamma_cross)
            f.trust_model.monitor_sudden(gamma_cross, D_pos, D_vel)

            f.trust_model.update_rating_vector(v_score, rating_type="local")
            trust_score = f.trust_model.calculate_trust_score(f.trust_model.rating_vector)

            adjusted_distance = f.trust_model.determine_following_distance(
                trust_score, ds=f.default_safe_distance, dacc=2.0
            )

            speed_cmd = max(0.0, min(f.max_speed, 0.5 * (distance - adjusted_distance)))

            print(f"[Trust] Score: {trust_score:.2f} | Dist: {distance:.2f} | Target: {adjusted_distance:.2f} | Speed: {speed_cmd:.2f}")

            try:
                follower.set_velocity_and_request_state(
                    forward=0.3,
                    turn=steering_cmd,
                    headlights=False,
                    leftTurnSignal=False,
                    rightTurnSignal=False,
                    brakeSignal=False,
                    reverseSignal=False
                )
            except Exception as e:
                print("[ControllerFollower] Command failed:", e)

            time.sleep(1)
        else:
            print("[Follower] Waiting for leader state...")
            

except KeyboardInterrupt:
    print("Stopping follower...")

finally:
    # Save trust data
    trust_data = np.array(f.trust_log)
    np.savez("logs/trust_validation.npz", trust_data=trust_data)
    print("[INFO] Trust log saved to logs/trust_validation.npz")

    f.stop()
    qlabs.close()



# while f.running == True:
#     f.follow(leader)
#     time.sleep(0.05)
    

# # # f = Follower(qcar=follower, idm_controller=idm)
# # f.run()
# try:
#     f.run()
# except KeyboardInterrupt:
#     print("Stopping follower...")
# finally:
#     # Save trust data
#     trust_data = np.array(f.trust_log)
#     np.savez("logs/trust_validation.npz", trust_data=trust_data)
#     print("[INFO] Trust log saved to logs/trust_validation.npz")

#     f.stop()
#     qlabs.close()

# # # Cleanup
# qlabs.close()
