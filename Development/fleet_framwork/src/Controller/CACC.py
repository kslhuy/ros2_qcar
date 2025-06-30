import numpy as np
import math

class CACC:
    def __init__(self, controller):
        self.controller = controller
        self.param_opt = controller.param_opt
        self.param_sys = controller.param_sys
        self.goal = controller.goal
        self.straightlane = controller.straightlane
        self.vehicle_id = controller.vehicle_id

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
            print(self.vehicle_id)
            acc = 0  # No car to follow
        else:
            u_coop = np.zeros(2)
            for car_j in surrounding_vehicles:
                # print(car_j)
                if type_state == "true":
                    x_j = car_j.state[0]
                    y_j = car_j.state[1]
                    v_j = car_j.state[3]
                else:
                    est = self.controller.vehicle.observer.est_global_state_current
                    x_j = est[0, car_j.vehicle_ic]
                    y_j = est[1, car_j.vehicle_ic]
                    v_j = est[3, car_j.vehicle_id]

                pos_j = np.array([x_j, y_j])
                d_vec = pos_j - pos
                s = np.linalg.norm(d_vec)
                if s > 1e-6:
                    dir_vec = d_vec / s
                else:
                    dir_vec = np.array([1.0, 0.0])  # fallback if positions are nearly identical

                spacing_target = ((host_car_id - car_j.vehicle_id) * s0 + h * v)
                spacing_error = s - spacing_target
                velocity_error = v_j - v

                u_coop += K @ np.array([spacing_error, velocity_error])
                
            u_coop /= num_vehicles
            acc = u_coop[0]  # Cooperative acceleration
            acc = max(-30,min(acc,30))

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
