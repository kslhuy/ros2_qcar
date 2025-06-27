import math
import numpy as np
import threading
import time
import socket
import pickle
import random
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

class GPSSync:
    """Simulates GPS-based time synchronization for vehicles."""
    def __init__(self):
        self.gps_time_offset = 0  # Offset between GPS and local time
        self.last_sync_time = time.time()  # Last GPS sync

    def get_gps_time(self):
        """Simulated function to get GPS time."""
        simulated_gps_time = time.time() + 2  # GPS is 2 sec ahead
        return simulated_gps_time

    def sync_with_gps(self):
        """Sync local clock with GPS time."""
        gps_time = self.get_gps_time()
        local_time = time.time()
        self.gps_time_offset = gps_time - local_time
        self.last_sync_time = local_time
        print(f"[GPS SYNC] GPS Time: {gps_time:.3f}, Local Time: {local_time:.3f}, Offset: {self.gps_time_offset:.3f} sec")

    def get_synced_time(self):
        """Returns corrected local time using GPS offset."""
        return time.time() + self.gps_time_offset

class Vehicle:
    """Represents a vehicle in a platoon, supporting leader or follower roles."""
    def __init__(self, qcar, idm_controller, vehicle_id, is_leader=False, max_steering=0.6, ip='127.0.0.1', send_port=5050, recv_port=5005):
        self.qcar = qcar
        self.idm = idm_controller
        self.vehicle_id = vehicle_id
        self.is_leader = is_leader
        self.max_steering = max_steering
        self.gps_sync = GPSSync()
        self.running = False
        self.thread = None

        # Communication setup
        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock.bind(('0.0.0.0', recv_port))
        self.target_ip = ip
        self.send_port = send_port
        self.recv_port = recv_port
        self.leader_state = {'pos': [0, 0, 0], 'rot': [0, 0, 0], 'v': 0.}
        self.last_seq = -1
        self.sequence_number = 0

        # Start communication threads
        threading.Thread(target=self.send_state, daemon=True).start()
        threading.Thread(target=self.receive_state, daemon=True).start()

    def send_state(self):
        """Sends vehicle state with simulated network delay."""
        while self.running:
            try:
                _, pos, rot, _ = self.qcar.get_world_transform()
                v = self.qcar.motorTach if hasattr(self.qcar, 'motorTach') else 0.5
                timestamp = self.gps_sync.get_synced_time()
                data = {
                    'seq': self.sequence_number,
                    'id': self.vehicle_id,
                    'pos': pos,
                    'rot': rot,
                    'v': v,
                    'timestamp': timestamp
                }
                network_delay = random.uniform(0.05, 0.15)  # 50-150ms delay
                time.sleep(network_delay)
                self.send_sock.sendto(pickle.dumps(data), (self.target_ip, self.send_port))
                print(f"[V{self.vehicle_id} SENT] Seq: {self.sequence_number}, Delay: {network_delay:.3f} sec")
                self.sequence_number += 1
            except Exception as e:
                print(f"[V{self.vehicle_id} SEND ERROR]: {e}")
                time.sleep(1.0)
                continue
            time.sleep(0.1)

    def receive_state(self):
        """Receives state from other vehicles."""
        while self.running:
            try:
                data, _ = self.recv_sock.recvfrom(1024)
                incoming = pickle.loads(data)
                if incoming['id'] != self.vehicle_id:  # Ignore own messages
                    seq = incoming.get('seq', -1)
                    if self.last_seq != -1:
                        missed = seq - self.last_seq - 1
                        print(f"[V{self.vehicle_id} RECEIVED] Seq: {seq}, Last: {self.last_seq}, Missed: {missed}")
                    else:
                        print(f"[V{self.vehicle_id} RECEIVED] Seq: {seq} (initial packet)")
                    self.last_seq = seq
                    self.leader_state = incoming
            except Exception as e:
                print(f"[V{self.vehicle_id} RECEIVE ERROR]: {e}")
                continue

    def wrap_to_pi(self, angle):
        """Wraps angle to [-pi, pi]."""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def update_movement(self):
        """Updates vehicle movement based on role (leader or follower)."""
        if self.is_leader:
            # Leader moves at constant velocity
            # speed_cmd = 0.5
            # steering_cmd = 0.0
            pass
        else:
            # Follower uses IDM/CACC to track leader
            try:
                _, pos_follower, rot_follower, _ = self.qcar.get_world_transform()
            except Exception as e:
                print(f"[V{self.vehicle_id} READ ERROR]: {e}")
                return

            pos_leader = self.leader_state['pos']
            rot_leader = self.leader_state['rot']
            v_leader = self.leader_state['v']

            # Pure pursuit for steering
            lookahead_distance = 0.4
            target_x = pos_leader[0] - lookahead_distance * math.cos(rot_leader[2])
            target_y = pos_leader[1] - lookahead_distance * math.sin(rot_leader[2])
            dx = target_x - pos_follower[0]
            dy = target_y - pos_follower[1]
            follower_heading = rot_follower[2]
            target_angle = math.atan2(dy, dx)
            heading_error = self.wrap_to_pi(target_angle - follower_heading)
            steering_cmd = -2.0 * heading_error
            steering_cmd = max(-self.max_steering, min(self.max_steering, steering_cmd))

            # IDM/CACC for speed
            v_follower = self.qcar.motorTach if hasattr(self.qcar, 'motorTach') else 0.5
            follower_state = [pos_follower[0], pos_follower[1], follower_heading, v_follower]
            class DummyVehicle:
                def __init__(self, state, vehicle_number=0):
                    self.state = state
                    self.vehicle_number = vehicle_number
            leader_state = [pos_leader[0], pos_leader[1], rot_leader[2], v_leader]
            dummy_leader = DummyVehicle(leader_state, vehicle_number=0)
            self.idm.controller.get_surrounding_vehicles = lambda *args, **kwargs: (None, [dummy_leader], None, None)

            _, input_u, _ = self.idm.get_optimal_input(
                host_car_id=self.vehicle_id,
                state=follower_state,
                last_input=None,
                lane_id=None,
                input_log=None,
                initial_lane_id=None,
                direction_flag=None,
                type_state="true",
                acc_flag=0
            )
            speed_cmd = max(0, input_u[0])

        # Apply control commands
        self.qcar.set_velocity_and_request_state(
            forward=speed_cmd,
            turn=steering_cmd,
            headlights=False,
            leftTurnSignal=False,
            rightTurnSignal=False,
            brakeSignal=False,
            reverseSignal=False
        )

    def run(self):
        """Main control loop for the vehicle."""
        self.running = True
        self.gps_sync.sync_with_gps()  # Initial GPS sync
        while self.running:
            if self.is_leader and int(time.time()) % 5 == 0:
                self.gps_sync.sync_with_gps()  # Leader syncs GPS every 5 seconds
            self.update_movement()
            time.sleep(0.2)

    def start(self):
        """Start the vehicle loop in a new thread."""
        if self.thread is None or not self.thread.is_alive():
            self.thread = threading.Thread(target=self.run, daemon=True)
            self.thread.start()

    def stop(self):
        """Stop the vehicle and clean up."""
        self.running = False
        if self.thread is not None:
            self.thread.join()
        self.send_sock.close()
        self.recv_sock.close()

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

    time.sleep(1)

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
    leader_vehicle = Vehicle(qcar=leader, idm_controller=control_algo, vehicle_id=0, is_leader=True, send_port=5005, recv_port=5050)
    follower_vehicle = Vehicle(qcar=follower, idm_controller=control_algo, vehicle_id=1, is_leader=False, send_port=5050, recv_port=5005)

    # Start vehicles
    leader_vehicle.start()
    follower_vehicle.start()

    # Run simulation for 10 seconds
    time.sleep(40)

    # Stop vehicles
    leader_vehicle.stop()
    follower_vehicle.stop()

    qlabs.close()
    print("Simulation ended.")

if __name__ == "__main__":
    main()