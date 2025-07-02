import numpy as np
import math

class CACC:
    def __init__(self, controller):
        self.controller = controller
        self.param_opt = controller.param_opt
        self.param_sys = controller.param_sys
        self.goal = controller.goal
        self.straightlane = controller.straightlane
        self.vehicle_number = controller.vehicle_number

    def get_optimal_input(self, host_car_id, state, last_input, lane_id, input_log,
                          initial_lane_id, direction_flag, type_state, acc_flag):
        acc_flag = 0

        # Extract vehicle parameters
        alpha = self.param_opt['alpha']
        beta = self.param_opt['beta']
        v0 = self.param_opt['v0']
        delta_exp = self.param_opt['delta']
        T = self.param_opt['T']
        s0 = self.param_opt['ri']
        # print('[cacc]',s0)
        K = self.param_opt['K']
        h = self.param_opt['hi']

        # Unpack current state
        x, y, theta, v = self.unpack_state(state)
        pos = np.array([x, y])

        # Get leading vehicles
        _, surrounding_vehicles, _, _ = self.controller.get_surrounding_vehicles(
            x, lane_id, direction_flag, host_car_id)

        num_vehicles = len(surrounding_vehicles)

        if num_vehicles == 0:
            print("No car ahead")
            print(self.vehicle_number)
            acc = 0  # No car to follow
        else:
            u_coop = np.zeros(2)
            for car_j in surrounding_vehicles:
                # print('carj',car_j)
                if type_state == "true":
                    x_j = car_j.state[0]
                    y_j = car_j.state[1]
                    v_j = car_j.state[3]
                    # print('carj state',car_j.state)
                else:
                    est = self.controller.vehicle.observer.est_global_state_current
                    x_j = est[0, car_j.vehicle_number]
                    y_j = est[1, car_j.vehicle_number]
                    v_j = est[3, car_j.vehicle_number]

                pos_j = np.array([x_j, y_j])
                d_vec = pos_j - pos
                s = np.linalg.norm(d_vec)
                if s > 1e-6:
                    dir_vec = d_vec / s
                else:
                    dir_vec = np.array([1.0, 0.0])  # fallback if positions are nearly identical

                spacing_target = ((host_car_id - car_j.vehicle_number) * s0 + h * v)
                spacing_error = s - spacing_target
                # print('vj',v_j)
                # print('v',v)
                velocity_error = v_j - v

                u_coop += K @ np.array([spacing_error, velocity_error])

            u_coop /= num_vehicles
            acc = u_coop[0]  # Cooperative acceleration
            # print('spacing error',acc)

        # Constant steering
        delta = 0
        input_u = [acc, delta]
        e = 0  # Error placeholder
        return acc_flag, input_u, e

    def unpack_input(self, input_u):
        acc = input_u[0]
        beta = input_u[1]
        return acc, beta

    def unpack_state(self, state):
        x, y, psi, v = state
        return x, y, psi, v
