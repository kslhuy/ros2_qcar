import socket
import pickle
import random
import threading
import math
import time

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

        # Velocity calculation
        self.prev_pos = None
        self.prev_time = None
        self.velocity = 0.5  # Initial velocity estimate


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
        speed_cmd = 0.0  # Default to stop if undefined
        steering_cmd = 0.0  # Default to no steering

        if self.is_leader:
            # # Leader moves at constant velocity
            # speed_cmd = 0.5
            # steering_cmd = 0.0
            pass
        else:
            # Follower uses IDM/CACC to track leader
            try:
                _, pos_follower, rot_follower, _ = self.qcar.get_world_transform()
            except Exception as e:
                print(f"[V{self.vehicle_id} READ ERROR]: {e}")
                # Use default values (stop) and skip control application
                self.qcar.set_velocity_and_request_state(
                    forward=speed_cmd,
                    turn=steering_cmd,
                    headlights=False,
                    leftTurnSignal=False,
                    rightTurnSignal=False,
                    brakeSignal=False,
                    reverseSignal=False
                )
                return
            
            # Calculate velocity using position difference
            current_time = time.time()
            if self.prev_pos is not None and self.prev_time is not None:
                # Compute Euclidean distance between current and previous position
                dx = pos_follower[0] - self.prev_pos[0]
                dy = pos_follower[1] - self.prev_pos[1]
                distance = math.sqrt(dx**2 + dy**2)
                dt = current_time - self.prev_time
                if dt > 0:  # Avoid division by zero
                    self.velocity = distance / dt
                else:
                    self.velocity = 0.0
            self.prev_pos = pos_follower
            self.prev_time = current_time


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
            v_follower = self.velocity
            print('v_follo',v_follower)
            follower_state = [pos_follower[0], pos_follower[1], follower_heading, v_follower]
            class DummyVehicle:
                def __init__(self, state, vehicle_number=0):
                    self.state = state
                    self.vehicle_number = vehicle_number
            leader_state = [pos_leader[0], pos_leader[1], rot_leader[2], v_leader]
            dummy_leader = DummyVehicle(leader_state, vehicle_number=0)
            self.idm.controller.get_surrounding_vehicles = lambda *args, **kwargs: (None, [dummy_leader], None, None)


            class DummyVehicle:
                def __init__(self, state, vehicle_number=0):
                    self.state = state
                    self.vehicle_number = vehicle_number
            leader_state = [pos_leader[0], pos_leader[1], rot_leader[2], v_leader]
            dummy_leader = DummyVehicle(leader_state, vehicle_number=0)
            self.idm.controller.get_surrounding_vehicles = lambda *args, **kwargs: (None, [dummy_leader], None, None)

            try:
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
            except Exception as e:
                print(f"[V{self.vehicle_id} IDM/CACC ERROR]: {e}")
                speed_cmd = 0.0  # Stop if controller fails

        # Apply control commands
        try:
            self.qcar.set_velocity_and_request_state(
                forward=speed_cmd,
                turn=steering_cmd,
                headlights=False,
                leftTurnSignal=False,
                rightTurnSignal=False,
                brakeSignal=False,
                reverseSignal=False
            )
        except Exception as e:
            print(f"[V{self.vehicle_id} CONTROL ERROR]: {e}")

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