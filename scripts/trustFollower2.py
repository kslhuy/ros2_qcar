# trustFollower2.py
import math
import numpy as np
import threading
import time
from Controller.idm_control import IDMControl

from communication import Comm


# class Comm:
#     def __init__(self, controller):
#         self.controller = controller
#     def get_local_state(self, id, host_id):
#         state = self.controller.state if id == self.controller.vehicle_number else np.zeros(4)
#         # print(f"[Comm] get_local_state(id={id}, host_id={host_id}) returning shape={np.shape(state)}")
#         # print(f"[Comm] get_local_state(id={id}, host_id={host_id}) state={state}")

#         return state
#     def get_global_state(self, id, host_id):
#         state = self.controller.observer.est_global_state_current
#         # print(f"[Comm] get_global_state(id={id}, host_id={host_id}) returning shape={np.shape(state)}")
#         # print(f"[Comm] get_global_state(id={id}, host_id={host_id}) state={state}")
#         return state
#     def get_input(self, id):
#         input_val = self.controller.input
#         print(f"[Comm] get_input(id={id}) returning shape={np.shape(input_val)}")
#         return input_val

class Follower:
    def __init__(self, qcar, idm_controller, vehicle_id=1, max_steering=0.6, trust_model=None):
        self.qcar = qcar
        self.idm = idm_controller
        self.max_steering = max_steering
        self.id = vehicle_id
        self.trust_model = trust_model
        self.running = False
        self.thread = None
        self.time_step = 0

    def wrap_to_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def follow(self, leader):
        try:
            _, pos_leader, rot_leader, _ = leader.get_world_transform()
            _, pos_follower, rot_follower, _ = self.qcar.get_world_transform()
        except Exception as e:
            print(f"[Follower-{self.id}] get_world_transform failed: {e}")
            return
        

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

        # Estimate leader velocity based on position difference
        if not hasattr(self, 'prev_pos_leader'):
            self.prev_pos_leader = pos_leader

        dx = pos_leader[0] - self.prev_pos_leader[0]
        dy = pos_leader[1] - self.prev_pos_leader[1]
        dt = 0.2  # control loop time step
        v_leader = math.hypot(dx, dy) / dt
        print('v leader',v_leader)
        self.prev_pos_leader = pos_leader
        
        dx_truth = pos_leader[0] - pos_follower[0]
        dy_truth = pos_leader[1] - pos_follower[1]
        distance = math.hypot(dx_truth, dy_truth)
        print('distance m',distance)

        v_follower = self.qcar.motorTach if hasattr(self.qcar, 'motorTach') else 0.5
        # v_leader = leader.motorTach if hasattr(leader, 'motorTach') else 0.5





        a_follower = 0
        a_leader = 0

        follower_state = np.array([pos_follower[0], pos_follower[1], follower_heading, v_follower])
        leader_state = np.array([pos_leader[0], pos_leader[1], rot_leader[2], v_leader])
        self.idm.controller.update_state(pos_follower, rot_follower, v_follower, a_follower)

        leader_controller = type('Controller', (), {
            'vehicle_number': self.id - 1,
            'state': leader_state,
            'input': np.array([a_leader]),
            'param': type('Param', (), {'l_r': 0.1})(),
            'dt': 0.05,
            'scenarios_config': type('Config', (), {
                'Monitor_sudden_change': True,
                'Dichiret_type': 'Dual'
            })(),
            'observer': type('Observer', (), {
                'est_local_state_current': leader_state,
                'est_global_state_current': np.zeros((4, 3))
            })(),
            'update_state': lambda self, pos, rot, v, a: (
                setattr(self, 'state', np.array([pos[0], pos[1], rot[2], v])),
                setattr(self.observer, 'est_local_state_current', self.state),
                self.observer.est_global_state_current[:, self.vehicle_number].__setitem__(slice(None), self.state),
                setattr(self, 'input', np.array([a]))
            )
        })()
        # leader_controller.center_communication = Comm(leader_controller)

        try:
            leader_controller.update_state(pos_leader, rot_leader, v_leader, a_leader)
        except Exception as e:
            print(f"[Follower-{self.id}] leader_controller.update_state failed: {e}")
            return

        trust_score = 1.0
        if self.trust_model:
            try:
                # print(f"[Follower-{self.id}] Calculating trust for vehicle_number={leader_controller.vehicle_number}")
                # final_score, _, _, _, _, _, _ = self.trust_model.calculateTrust(
                #     host_vehicle=self.idm.controller,
                #     target_vehicle=leader_controller,
                #     leader_vehicle=leader_controller,
                #     neighbors=[leader_controller],
                #     is_nearby=True,
                #     instant_idx=self.time_step
                # )
                final_score, _, _, _, _, _, _ = self.trust_model.calculateTrust(
                    host_vehicle=self.idm.controller,
                    target_vehicle=self.idm.controller.center_communication.vehicles[self.id - 1],
                    leader_vehicle=self.idm.controller.center_communication.vehicles[self.id - 1],
                    neighbors=[leader_controller],
                    is_nearby=True,
                    instant_idx=self.time_step
                )

                trust_score = final_score
                # print('trust',trust_score)
                ds = self.idm.controller.param_opt['ri']
                # print('ds',ds)
                dacc = ds * 2
                # self.idm.controller.param_opt['s0'] = self.trust_model.determine_following_distance(trust_score, ds, dacc)
                ds = self.trust_model.determine_following_distance(trust_score, ds, dacc)
                print('following distance',ds)
                self.time_step += 1
            except Exception as e:
                print(f"[Follower-{self.id}] Trust model failed: {e}")
                self.idm.controller.param_opt['s0'] = self.idm.controller.param_opt['s0']

        class DummyVehicle:
            def __init__(self, state, vehicle_number=0):
                self.state = state
                self.vehicle_number = vehicle_number

        dummy_leader = DummyVehicle(leader_state, vehicle_number=self.id - 1)
        self.idm.controller.get_surrounding_vehicles = lambda *args, **kwargs: (None, [dummy_leader], None, None)

        try:
            _, input_u, _ = self.idm.get_optimal_input(
                host_car_id=self.id,
                state=follower_state,
                last_input=None,
                lane_id=None,
                input_log=None,
                initial_lane_id=None,
                direction_flag=None,
                type_state="true",
                acc_flag=0
            )
        except Exception as e:
            print(f"[Follower-{self.id}] get_optimal_input failed: {e}")
            return

        speed_cmd = max(0, input_u[0])

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
            print(f"[Follower-{self.id}] set_velocity_and_request_state failed: {e}")

    def run(self, leader):
        self.running = True
        while self.running:
            self.follow(leader)
            time.sleep(0.05)

    def start(self, leader):
        if self.thread is None or not self.thread.is_alive():
            self.thread = threading.Thread(target=self.run, args=(leader,))
            self.thread.start()

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join()