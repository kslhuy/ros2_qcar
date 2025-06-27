import numpy as np
import math

class LookAheadControl:
    def __init__(self, controller):
        self.controller = controller
        self.param_opt = controller.param_opt
        self.param_sys = controller.param_sys
        self.goal = controller.goal
        self.straightlane = controller.straightlane
        self.vehicle_number = controller.vehicle_number

    def get_optimal_input(self, host_car_id, state, last_input, lane_id,
                          input_log, initial_lane_id, direction_flag, type_state, acc_flag):
        acc_flag = 0

        l_r = self.param_sys['l_r']
        l_f = self.param_sys['l_f']
        C1 = self.param_sys['C1']
        C2 = self.param_sys['C2']
        mass = self.param_sys['mass']

        hi = self.param_opt['hi']
        ri = self.param_opt['ri']
        k1 = 2
        k2 = 1.5

        x, y, theta, v = self.unpack_state(state)

        # car_fc, _, _ = self.controller.get_surrounding_vehicles(x, lane_id, direction_flag, host_car_id)

        _, car_fss, _, _ = self.controller.get_surrounding_vehicles(...)
        car_fc = car_fss[-1] if car_fss else None


        if car_fc is None:
            print("No car preceding")
            print(self.vehicle_number)
            return 0, [0, 0], 0

        if type_state == "true":
            x_lead, y_lead, theta_lead, v_lead = self.unpack_state(car_fc.state)
        else:
            est_state = self.controller.vehicle.observer.est_global_state_current[:, car_fc.vehicle_number]
            x_lead, y_lead, theta_lead, v_lead = self.unpack_state(est_state)

            x_lead_true, y_lead_true, theta_lead_true, v_lead_true = self.unpack_state(car_fc.state)
            meas_rel_pos = np.hypot(x_lead_true - x, y_lead_true - y)
            meas_rel_vel = v_lead_true - v
            est_rel_pos = np.hypot(x_lead - x, y_lead - y)
            est_rel_vel = v_lead - v

            if (meas_rel_pos - est_rel_pos) / meas_rel_pos < 0 or \
               (meas_rel_vel - est_rel_vel) / meas_rel_vel > 0.1:
                print("The relative state is not estimated correctly")
                x_lead, y_lead, theta_lead, v_lead = self.unpack_state(car_fc.state)

        kappa_lead = self.compute_curvature(v_lead, theta_lead)
        s_bar = self.compute_s_bar(kappa_lead, ri, hi, v)
        sx_lead = s_bar * math.sin(theta_lead)
        sy_lead = -s_bar * math.cos(theta_lead)

        z1 = x_lead + sx_lead - x - (ri + hi * v) * math.cos(theta)
        z2 = y_lead + sy_lead - y - (ri + hi * v) * math.sin(theta)
        z3 = v_lead * math.cos(theta_lead) - v * math.cos(theta)
        z4 = v_lead * math.sin(theta_lead) - v * math.sin(theta)

        acc = k1 * z1 + z3
        acc = max(.0, min(acc, 1.0))  # Clip between -1 m/s² and 1 m/s²

        omega = k2 * z2 + z4

        delta = -math.atan(omega * l_r / max(v, 0.1))
        # Alternatively:
        # wheelbase = l_r + l_f
        # delta = wheelbase * omega / max(v, 0.1) - mass * omega * v / wheelbase * (l_f / C2 - l_r / C1)
        print(f"z2: {z2:.2f}, omega: {omega:.2f}, delta: {delta:.2f}")


        input_u = [acc, delta]
        e = 0  # or [z1, z2, z3, z4] if needed
        # e = [z1, z2, z3, z4]
        return acc_flag, input_u, e

    def unpack_state(self, state):
        x = state[0]
        y = state[1]
        theta = state[2]
        v = state[3]
        return x, y, theta, v

    def compute_curvature(self, v, theta):
        return theta / max(v, 0.1)

    def compute_s_bar(self, kappa, ri, hi, v):
        if kappa == 0:
            return 0
        else:
            return (-1 + math.sqrt(1 + kappa**2 * (ri + hi * v)**2)) / kappa
