import numpy as np
import matplotlib.pyplot as plt
import csv
import os

class TriPTrustModel:
    def __init__(self, k=5, wt=0.5, C=0.2):
        self.k = k
        self.wt = wt
        self.C = C
        self.wv = 2.0
        self.wd = 1.0
        self.wa = 1.0
        self.wj = 1.0
        self.tacc = 1.2
        self.sigma2 = 1
        self.tau2 = 0.5
        self.last_d = 20
        self.w = 10
        self.Threshold_anomalie = 3
        self.reduce_factor = 0.5
        self.rating_vector = np.zeros(self.k)
        self.rating_vector[-1] = 1.0
        self.rating_vector_global = np.zeros(self.k)
        self.rating_vector_global[-1] = 1.0
        self.trust_sample_log = []
        self.gamma_cross_log = []
        self.gamma_local_log = []
        self.v_score_log = []
        self.d_score_log = []
        self.a_score_log = []
        self.beacon_score_log = []
        self.final_score_log = []
        self.flag_taget_attk_log = []
        self.flag_glob_est_check_log = []
        self.flag_local_est_check_log = []
        self.flag_taget_attk = False
        self.flag_glob_est_check = False
        self.flag_local_est_check = False
        self.lead_state_lastest = np.zeros(4)
        self.lead_state_lastest_timestamp = 0
        self.D_pos_log = []
        self.D_vel_log = []
        self.anomaly_pos_log = []
        self.anomaly_vel_log = []
        self.anomaly_gamma_log = []

    def evaluate_velocity(self, host_id, target_id, v_y, v_host, v_leader, a_leader, b_leader, tolerance=0.1):
        v_ref = v_leader + b_leader * a_leader
        alpha = 0.9 if (host_id - target_id) > 0 else 0.1
        if v_ref == 0 or v_leader == 0 or np.sign(v_ref * v_leader) < 0 or np.sign(v_ref * v_host) < 0:
            return max(1 - abs(v_y), 0)
        deviation_ref = abs(v_y - v_ref) / v_ref
        deviation_host = abs(v_y - v_host) / v_host
        v_score_ref = 1.0 if deviation_ref <= tolerance else max(1 - (deviation_ref - tolerance) / (1 - tolerance), 0)
        v_score_host = max(1 - deviation_host, 0)
        return (1 - alpha) * v_score_ref + alpha * v_score_host

    def evaluate_distance(self, d_y, d_measured):
        # return max(1 - abs((d_y - d_measured) / d_measured), 0)
        # return max(1 - abs((d_y - d_measured) / d_measured) if d_measured != 0 else 0, 0)
        # In evaluate_distance
        return max(1 - abs((d_y - d_measured) / d_measured) * 0.5, 0)


    def evaluate_acceleration(self, a_y, a_host, d, ts):
        v_rel = (d[0] - d[1]) / ts
        a_diff = a_y - a_host
        return max(1 - abs(v_rel / ts * a_diff), 0)

    def evaluate_jerkiness(self, j_y, j_thresh=2.0):
        if abs(j_y) > j_thresh:
            return min(j_thresh / abs(j_y), 1)
        else:
            return 1

    def evaluate_beacon_timeout(self, beacon_received):
        return 1 if beacon_received else 0
    
    # def h_score = evaluate_heading 

    def calculate_trust_sample_wo_Acc(self, v_score, d_score, a_score, beacon_score, is_nearby):
        if is_nearby:
            return beacon_score * (v_score ** self.wv) * (d_score ** self.wd)
        else:
            return beacon_score * (v_score ** self.wv)

    def calculate_trust_sample_w_Jek(self, v_score, d_score, a_score, j_score, beacon_score):
        return beacon_score * (v_score ** self.wv) * (d_score ** self.wd) * (a_score ** self.wa) * (j_score ** self.wj)
    
    # def calculate_trust_sample_normal

    def calculate_trust_sample(self, v_score, d_score, a_score, beacon_score, is_nearby):
        if is_nearby:
            return beacon_score * (v_score ** self.wv) * (d_score ** self.wd) * (a_score ** self.wa)
        else:
            return beacon_score * (v_score ** self.wv) * (a_score ** self.wa)

    def update_rating_vector(self, trust_sample, rating_type="local"):
        trust_level = min(int(round(trust_sample * (self.k - 1))) + 1, self.k)
        trust_vector = np.zeros(self.k)
        trust_vector[trust_level - 1] = 1
        # if rating_type == "local":
        #     current_score = self.calculate_trust_score(self.rating_vector)
        #     lambda_y = current_score * self.wt
        #     self.rating_vector = (1 - lambda_y) * self.rating_vector + trust_vector
        # else:
        #     current_score = self.calculate_trust_score(self.rating_vector_global)
        #     lambda_y = current_score * self.wt
        #     self.rating_vector_global = (1 - lambda_y) * self.rating_vector_global + trust_vector

        if rating_type == "local":
            current_score = self.calculate_trust_score(self.rating_vector)
        else:
            current_score = self.calculate_trust_score(self.rating_vector_global)

        lambda_y = current_score * self.wt

        if rating_type == "local":
            self.rating_vector = (1 - lambda_y) * self.rating_vector + trust_vector
        else:
            self.rating_vector_global = (1 - lambda_y) * self.rating_vector_global + trust_vector


    def calculate_trust_score(self, rating_vector):
        S_y = (rating_vector + self.C / self.k) / (self.C + np.sum(rating_vector))
        epsilon = 0.01
        weights = (np.arange(self.k) + epsilon) / (self.k - 1 + epsilon)
        return np.dot(weights, S_y)

    def compute_cross_host_target_factor(self, host_id, host_vehicle, target_id, target_vehicle):
        host_global_estimate = host_vehicle.observer.est_global_state_current
        target_global_estimate = target_vehicle.center_communication.get_global_state(target_id, host_id)
        # print(f"[TriPTrustModel] host_global_estimate shape={host_global_estimate.shape}")
        # print(f"[TriPTrustModel] target_global_estimate shape={target_global_estimate.shape}")
        # sigma2_diag_element = [2, 1, 0.01, 0.5]
        sigma2_diag_element = [50, 50, 1, 12]
        sigma2_matrix = np.diag(sigma2_diag_element)
        sigma2_pos = np.diag([2, 1])
        sigma2_vel = np.diag([0.5])
        D = 0
        D_pos = 0
        D_vel = 0
        num_vehicles = target_global_estimate.shape[1]
        for j in range(num_vehicles):
            pos_diff = target_global_estimate[0:2, j] - host_global_estimate[0:2, j]
            # print(f"[TriPTrustModel] pos_diff shape={pos_diff.shape}")
            D_pos += pos_diff.T @ np.linalg.inv(sigma2_pos) @ pos_diff
            vel_diff = np.array([target_global_estimate[3, j] - host_global_estimate[3, j]])
            # print(f"[TriPTrustModel] vel_diff shape={vel_diff.shape}")
            D_vel += vel_diff.T @ np.linalg.inv(sigma2_vel) @ vel_diff
            x_diff = target_global_estimate[:, j] - host_global_estimate[:, j]
            # print('xdiff',x_diff)
            # print(f"[TriPTrustModel] x_diff shape={x_diff.shape}")
            D += x_diff.T @ np.linalg.inv(sigma2_matrix) @ x_diff
            # print(f"[TriPTrustModel] D={D}")
        gamma_cross = np.exp(-D)
        return gamma_cross, D_pos, D_vel

    def compute_local_consistency_factor(self, host_vehicle, target_vehicle, neighbors):
        half_length_vehicle = target_vehicle.param.l_r
        M_i = 0
        predecessor = None
        successor = None
        host_id = host_vehicle.vehicle_number
        target_global_estimate = target_vehicle.center_communication.get_global_state(target_vehicle.vehicle_number, host_id)
        # x_l_i = target_global_estimate[[0, 3], host_id]
        x_l_i = target_global_estimate[0:2, host_id]  # [x, y]
        # print(f"[TriPTrustModel] x_l_i shape={x_l_i.shape}")
        # tau2_diag_element = [2, 0.5]
        tau2_diag_element = [10, 5]
        tau2_matrix = np.diag(tau2_diag_element)
        E = 0
        for neighbor in neighbors:
            car_idx = neighbor.vehicle_number
            if abs(car_idx - host_vehicle.vehicle_number) == 1:
                if car_idx > host_vehicle.vehicle_number:
                    M_i += 1
                    successor = neighbor
                else:
                    M_i += 1
                    predecessor = neighbor
        if not predecessor and not successor:
            return 1
        if predecessor:
            host_distance_measurement = (predecessor.state[0] - host_vehicle.state[0]) - half_length_vehicle
            velocity_diff = abs(predecessor.state[3] - host_vehicle.state[3])
            y_i_pred = np.array([host_distance_measurement, velocity_diff])
            pred_id = predecessor.vehicle_number
            # x_l_pred = target_global_estimate[[0, 3], pred_id]
            # rel_state_est = abs(x_l_pred - x_l_i - np.array([half_length_vehicle, 0]))
            x_l_pred = target_global_estimate[0:2, pred_id]  # [x, y]
            rel_state_est = abs(x_l_pred - x_l_i - np.array([half_length_vehicle, 0]))
            e = rel_state_est - y_i_pred
            # print(f"[TriPTrustModel] e shape (predecessor)={e.shape}")
            E += e.T @ np.linalg.inv(tau2_matrix) @ e
        if successor:
            host_distance_measurement = (host_vehicle.state[0] - successor.state[0]) - half_length_vehicle
            velocity_diff = abs(successor.state[3] - host_vehicle.state[3])
            y_i_successor = np.array([host_distance_measurement, velocity_diff])
            successor_id = successor.vehicle_number
            # x_l_successor = target_global_estimate[[0, 3], successor_id]
            # rel_state_est = abs(x_l_i - x_l_successor - np.array([half_length_vehicle, 0]))
            x_l_successor = target_global_estimate[0:2, successor_id]  # [x, y]
            rel_state_est = abs(x_l_i - x_l_successor - np.array([half_length_vehicle, 0]))
            e = rel_state_est - y_i_successor
            # print(f"[TriPTrustModel] e shape (successor)={e.shape}")
            E += e.T @ np.linalg.inv(tau2_matrix) @ e
        gamma_local = np.exp(-E)
        return gamma_local

    def monitor_sudden(self, gamma_cross, D_pos, D_vel):
        anomaly_gamma = 0
        if len(self.gamma_cross_log) >= self.w:
            window = self.gamma_cross_log[-self.w:]
            mu = np.mean(window)
            sigma = np.std(window)
            if sigma > 0 and abs(gamma_cross - mu) > 2 * sigma:
                anomaly_gamma = 1
        self.anomaly_gamma_log.append(anomaly_gamma)
        self.D_pos_log.append(D_pos)
        self.D_vel_log.append(D_vel)
        anomaly_pos = 0
        if len(self.D_pos_log) >= self.w:
            window = self.D_pos_log[-self.w:]
            mu = np.mean(window)
            sigma = np.std(window)
            if sigma > 0 and abs(D_pos - mu) > 2 * sigma:
                anomaly_pos = 1
        self.anomaly_pos_log.append(anomaly_pos)
        anomaly_vel = 0
        if len(self.D_vel_log) >= self.w:
            window = self.D_vel_log[-self.w:]
            mu = np.mean(window)
            sigma = np.std(window)
            if sigma > 0 and abs(D_vel - mu) > 2 * sigma:
                anomaly_vel = 1
        self.anomaly_vel_log.append(anomaly_vel)
        beta = 1
        if len(self.anomaly_gamma_log) >= self.w:
            drop_packet_anomaly = self.w - 1 - sum(self.beacon_score_log[-(self.w - 1):])
            count_gamma = sum(self.anomaly_gamma_log[-self.w:])
            count_pos = sum(self.anomaly_pos_log[-self.w:])
            count_vel = sum(self.anomaly_vel_log[-self.w:])
            min_anomalies = min(count_gamma, count_pos, count_vel, drop_packet_anomaly)
            if min_anomalies > self.Threshold_anomalie:
                beta = 1 - self.reduce_factor * (min_anomalies / self.w)
            # print(f"[TriPTrustModel] monitor_sudden: count_gamma={count_gamma}, count_pos={count_pos}, count_vel={count_vel}, drop_packet_anomaly={drop_packet_anomaly}, beta={beta}")
        return beta

    def calculateTrust(self, host_vehicle, target_vehicle, leader_vehicle, neighbors, is_nearby, instant_idx):
        host_id = host_vehicle.vehicle_number
        target_id = target_vehicle.vehicle_number
        half_length_vehicle = target_vehicle.param.l_r
        leader_state = host_vehicle.center_communication.get_local_state(leader_vehicle.vehicle_number, host_id)
        print(f"[TriPTrustModel] leader_state ={leader_state}")
        # print(f"[TriPTrustModel] leader_state shape={leader_state.shape}")
        if np.any(np.isnan(leader_state)):
            leader_state = self.lead_state_lastest
            leader_beacon_interval = (instant_idx - self.lead_state_lastest_timestamp) * host_vehicle.dt
        else:
            self.lead_state_lastest = leader_state
            self.lead_state_lastest_timestamp = instant_idx
            leader_beacon_interval = 0
        leader_input = host_vehicle.center_communication.get_input(leader_vehicle.vehicle_number)
        # print(f"[TriPTrustModel] leader_input shape={leader_input.shape}")
        leader_velocity = leader_state[3]
        leader_acceleration = leader_input[0]
        print(f"[TriPTrustModel] leader_velocity ={leader_velocity}")
        print(f"[TriPTrustModel] leader_acceleration ={leader_acceleration}")
        # host_pos_X = host_vehicle.observer.est_local_state_current[0]##################### change to 2D 
        host_pos_X = host_vehicle.observer.est_local_state_current[0:2]
        host_velocity = host_vehicle.observer.est_local_state_current[3]
        host_acceleration = host_vehicle.input[0]
        

        target_pos_2d = target_vehicle.state[0:2]  # Assuming target_vehicle.state is [x, y, ..., v]
        pos_diff = host_pos_X - target_pos_2d
        # host_distance_measurement = np.linalg.norm(pos_diff) * np.sign(host_id - target_id) - half_length_vehicle
        host_distance_measurement = np.linalg.norm(pos_diff) - half_length_vehicle
        # print(f"[TriPTrustModel] host_pos_X={host_pos_X}, target_pos_2d={target_pos_2d}, host_distance_measurement={host_distance_measurement}")
        target_state = host_vehicle.center_communication.get_local_state(target_id, host_id)
        # host_distance_measurement = (host_id - target_id) * (target_vehicle.state[0] - host_pos_X) - half_length_vehicle
        # target_state = host_vehicle.center_communication.get_local_state(target_id, host_id)
        # print(f"[TriPTrustModel] target_state shape={target_state.shape}")
        if np.any(np.isnan(target_state)):
            beacon_score = 0
            v_score = 0
            d_score = 0
            a_score = 0
            local_trust_sample = 0
        else:
            beacon_score = self.evaluate_beacon_timeout(True)
            target_input = host_vehicle.center_communication.get_input(target_id)
            # print(f"[TriPTrustModel] target_input shape={target_input.shape}")
            # target_pos_X = target_state[0]
            target_pos_X = target_state[0:2]
            # print('target state',target_state)

            # Compute 2D distance difference
            pos_diff_reported = target_pos_X - host_pos_X
            target_reported_distance = np.linalg.norm(pos_diff_reported) * np.sign(host_id - target_id) - half_length_vehicle
            # target_reported_distance = (host_id - target_id) * (target_pos_X - host_pos_X) - half_length_vehicle
            # target_reported_distance = np.linalg.norm(pos_diff_reported) - half_length_vehicle  # Remove np.sign
            print(f"[TriPTrustModel] target_reported_distance={target_reported_distance}")
            target_reported_velocity = target_state[3]
            target_reported_acceleration = target_input[0]
            v_score = self.evaluate_velocity(host_id, target_id, target_reported_velocity, host_velocity,
                                            leader_velocity, leader_acceleration, leader_beacon_interval, 0.1)
            d_score = self.evaluate_distance(target_reported_distance, host_distance_measurement) if is_nearby else 0
            a_score = self.evaluate_acceleration(target_reported_acceleration, host_acceleration,
                                                [host_distance_measurement, self.last_d], host_vehicle.dt)
            self.last_d = host_distance_measurement
            # local_trust_sample = self.calculate_trust_sample_wo_Acc(v_score, d_score, a_score, beacon_score, is_nearby)######### calculate_trust_sample
            local_trust_sample = self.calculate_trust_sample(v_score, d_score, a_score, beacon_score, is_nearby)######### calculate_trust_sample
        # print(f"[TriPTrustModel] v_score={v_score}, d_score={d_score}, a_score={a_score}, beacon_score={beacon_score}, local_trust_sample={local_trust_sample}")

        if np.any(np.isnan(target_vehicle.center_communication.get_global_state(target_id, host_id))):
            gamma_cross = 0
            gamma_local = 0
            global_trust_sample = 0
            D_pos = 0
            D_vel = 0
        else:
            gamma_cross, D_pos, D_vel = self.compute_cross_host_target_factor(host_id, host_vehicle, target_id, target_vehicle)
            gamma_local = self.compute_local_consistency_factor(host_vehicle, target_vehicle, neighbors)
            global_trust_sample = gamma_cross * gamma_local
        beta = self.monitor_sudden(gamma_cross, D_pos, D_vel) if hasattr(host_vehicle, 'scenarios_config') and host_vehicle.scenarios_config.Monitor_sudden_change else 1
        # print(f"[TriPTrustModel] gamma_cross={gamma_cross}, gamma_local={gamma_local}, beta={beta}, global_trust_sample={global_trust_sample}")
        if hasattr(host_vehicle, 'scenarios_config') and host_vehicle.scenarios_config.Dichiret_type == "Single":
            trust_sample_ext = local_trust_sample * global_trust_sample
            self.update_rating_vector(trust_sample_ext, "local")
            final_score = self.calculate_trust_score(self.rating_vector)
        else:
            self.update_rating_vector(local_trust_sample, "local")
            local_trust_sample = self.calculate_trust_score(self.rating_vector)
            self.update_rating_vector(global_trust_sample, "global")
            global_trust_sample = self.calculate_trust_score(self.rating_vector_global)
            final_score = local_trust_sample * global_trust_sample
            # final_score = local_trust_sample * (gamma_cross ** 0.5) * (gamma_local ** 0.5)
            # final_score = local_trust_sample

        # final_score = final_score * beta
        final_score = max(final_score * beta, 0.01)

        self.flag_taget_attk = gamma_local > 0.5 and gamma_cross < 0.5
        self.flag_glob_est_check = gamma_local < 0.5 and gamma_cross > 0.5
        self.flag_local_est_check = local_trust_sample < 0.5
        self.trust_sample_log.append(local_trust_sample)
        self.gamma_cross_log.append(gamma_cross)
        self.gamma_local_log.append(gamma_local)
        self.v_score_log.append(v_score)
        self.d_score_log.append(d_score)
        self.a_score_log.append(a_score)
        self.beacon_score_log.append(beacon_score)
        self.final_score_log.append(final_score)
        self.flag_taget_attk_log.append(self.flag_taget_attk)
        self.flag_glob_est_check_log.append(self.flag_glob_est_check)
        self.flag_local_est_check_log.append(self.flag_local_est_check)
        
        # Save scores to CSV
        csv_filename = f"trust_scores_car_{host_id}_target_{target_id}.csv"
        csv_path = os.path.join("/home/qcar2_scripts", csv_filename)
        scores_row = {
            'time_step': instant_idx,
            'trust_sample': local_trust_sample,
            'gamma_cross': gamma_cross,
            'gamma_local': gamma_local,
            'v_score': v_score,
            'd_score': d_score,
            'a_score': a_score,
            'beacon_score': beacon_score,
            'final_score': final_score
        }
        file_exists = os.path.isfile(csv_path)
        with open(csv_path, 'a', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=scores_row.keys())
            if not file_exists:
                writer.writeheader()
            writer.writerow(scores_row)
        # print(f"[TriPTrustModel] Saved scores to {csv_path}")
        
        print(f"[TriPTrustModel] host_id={host_id}, target_id={target_id}, final_score={final_score}")
        return final_score, local_trust_sample, gamma_cross, v_score, d_score, a_score, beacon_score

    def plot_trust_log(self, nb_host_car, nb_target_car):
        plt.figure(figsize=(10, 6))
        plt.plot(self.trust_sample_log, label='Trust Sample', linewidth=1, linestyle='-', color='b')
        plt.plot(self.gamma_cross_log, label='Gamma Cross', linewidth=1, linestyle='--', color='r')
        plt.plot(self.gamma_local_log, label='Gamma Local', linewidth=1, linestyle='-.', color='g')
        plt.plot(self.v_score_log, label='V Score', linewidth=1, linestyle=':', color='c')
        plt.plot(self.d_score_log, label='D Score', linewidth=1, linestyle='-', color='m')
        plt.plot(self.a_score_log, label='A Score', linewidth=1, linestyle='--', color='y')
        plt.plot(self.final_score_log, label='Final Score', linewidth=1.5, linestyle='-', color='k')
        plt.xlabel('Time Step', fontsize=12)
        plt.ylabel('Value', fontsize=12)
        plt.title(f'Car {nb_host_car} -> Trust and Scores for Car {nb_target_car}', fontsize=14)
        plt.legend(loc='best', fontsize=10)
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.tight_layout()
        plot_filename = f"trust_plot_car_{nb_host_car}_target_{nb_target_car}.png"
        plot_path = os.path.join("/home/qcar2_scripts", plot_filename)
        plt.savefig(plot_path, dpi=300, bbox_inches='tight')
        plt.close()
        # print(f"[TriPTrustModel] Plot saved to {plot_path}")

    def determine_following_distance(self, trust_score, ds, dacc):
        if trust_score > 0.8:
            return ds
        elif trust_score > 0.2:
            return ds + (dacc - ds) * (0.8 - trust_score)
        else:
            return dacc