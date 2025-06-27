import numpy as np
from math import atan2, pi, sin, cos, exp

class TriPTrustModel:
    def __init__(self):
        self.wv = 1.0
        self.wd = 0.5
        self.wa = 0.5
        self.wj = 0.5
        self.wh = 0.5

        self.wv_nearby = 2.0
        self.wd_nearby = 1.0
        self.wa_nearby = 1.0
        self.wj_nearby = 1.0
        self.wh_nearby = 1.0

        self.wt = 0.5
        self.C = 0.2
        self.tacc = 1.2
        self.k = 5
        self.rating_vector = np.zeros(self.k)
        self.rating_vector[4] = 1.0
        self.rating_vector_global = np.zeros(self.k)
        self.rating_vector_global[4] = 1.0

        self.sigma2 = 1
        self.tau2 = 0.5
        self.last_d = 20

        self.trust_sample_log = []
        self.gamma_cross_log = []
        self.gamma_local_log = []
        self.v_score_log = []
        self.d_score_log = []
        self.a_score_log = []
        self.h_score_log = []
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

        self.w = 10
        self.Threshold_anomalie = 3
        self.reduce_factor = 0.5

        self.previous_state = np.zeros(4)

    def evaluate_velocity(self, host_id, target_id, v_y, v_host, v_leader, a_leader, b_leader, tolerance=0.1):
        v_ref = v_leader + b_leader * a_leader

        if (host_id - target_id) > 0:
            alpha = 0.9
        else:
            alpha = 0.1

        if v_ref == 0 or v_leader == 0 or np.sign(v_ref * v_leader) < 0 or np.sign(v_ref * v_host) < 0:
            return max(1 - abs(v_y), 0)

        deviation_ref = abs(v_y - v_ref) / v_ref
        deviation_host = abs(v_y - v_host) / v_host

        v_score_ref = 1.0 if deviation_ref <= tolerance else max(1 - (deviation_ref - tolerance) / (1 - tolerance), 0)
        v_score_host = max(1 - deviation_host, 0)

        return (1 - alpha) * v_score_ref + alpha * v_score_host

    def evaluate_distance(self, d_y, d_measured):
        return max(1 - abs((d_y - d_measured) / d_measured), 0)

    def evaluate_acceleration(self, a_y, a_host, d, ts):
        v_rel = (d[0] - d[1]) / ts
        a_diff = a_y - a_host
        return max(1 - abs((v_rel / d[0]) * a_diff), 0)

    def evaluate_jerkiness(self, j_y, j_thresh=2.0):
        if abs(j_y) > j_thresh:
            return min(j_thresh / abs(j_y), 1)
        else:
            return 1

    def evaluate_beacon_timeout(self, beacon_received):
        return 1 if beacon_received else 0

    def evaluate_heading(self, target_pos_X, target_pos_Y, reported_heading, instant_idx):
        if np.all(self.previous_state == 0):
            self.previous_state = np.array([target_pos_X, target_pos_Y, 0, 0])
            return 1

        prev_x, prev_y = self.previous_state[0], self.previous_state[1]
        delta_Y = target_pos_Y - prev_y
        delta_X = target_pos_X - prev_x
        theta_est = atan2(delta_Y, delta_X)
        theta_diff = abs(reported_heading - theta_est)
        theta_diff = min(theta_diff, 2 * pi - theta_diff)

        theta_max = pi / 18  # ~10 degrees
        h_score = max(1 - theta_diff / theta_max, 0)
        self.previous_state = np.array([target_pos_X, target_pos_Y, 0, 0])

        return h_score
    
    def calculate_trust_sample_wo_Acc(self, v_score, d_score, a_score, beacon_score, h_score, is_nearby):
        if is_nearby:
            return beacon_score * (v_score ** self.wv_nearby) * (d_score ** self.wd_nearby) * (h_score ** self.wh_nearby)
        else:
            return beacon_score * (v_score ** self.wv)

    def calculate_trust_sample_w_Jek(self, v_score, d_score, a_score, j_score, beacon_score):
        return beacon_score * (v_score ** self.wv) * (d_score ** self.wd) * (a_score ** self.wa) * (j_score ** self.wj)

    def calculate_trust_sample_normal(self, v_score, d_score, a_score, beacon_score, h_score, is_nearby):
        if is_nearby:
            return beacon_score * (v_score ** self.wv_nearby) * (d_score ** self.wd_nearby) * (h_score ** self.wh_nearby)
        else:
            return beacon_score * (v_score ** self.wv) * (d_score ** self.wd)

    def calculate_trust_sample(self, v_score, d_score, a_score, beacon_score, h_score, is_nearby):
        if is_nearby:
            return beacon_score * (v_score ** self.wv_nearby) * (d_score ** self.wd_nearby) * (a_score ** self.wa_nearby)
        else:
            return beacon_score * (v_score ** self.wv) * (a_score ** self.wa)

    def update_rating_vector(self, trust_sample, rating_type="local"):
        trust_level = min(int(round(trust_sample * (self.k - 1))) + 1, self.k)
        trust_vector = np.zeros(self.k)
        trust_vector[trust_level - 1] = 1

        if rating_type == "local":
            current_trust = self.calculate_trust_score(self.rating_vector)
            lambda_y = current_trust * self.wt
            self.rating_vector = (1 - lambda_y) * self.rating_vector + trust_vector
        else:
            current_trust = self.calculate_trust_score(self.rating_vector_global)
            lambda_y = current_trust * self.wt
            self.rating_vector_global = (1 - lambda_y) * self.rating_vector_global + trust_vector

    def calculate_trust_score(self, rating_vector):
        S_y = (rating_vector + self.C / self.k) / (self.C + np.sum(rating_vector))
        epsilon = 0.01
        weights = ((np.arange(self.k)) + epsilon) / (self.k - 1 + epsilon)
        return np.dot(weights, S_y)


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
            mu = np.mean(self.D_pos_log[-self.w:])
            sigma = np.std(self.D_pos_log[-self.w:])
            if sigma > 0 and abs(D_pos - mu) > 2 * sigma:
                anomaly_pos = 1
        self.anomaly_pos_log.append(anomaly_pos)

        anomaly_vel = 0
        if len(self.D_vel_log) >= self.w:
            mu = np.mean(self.D_vel_log[-self.w:])
            sigma = np.std(self.D_vel_log[-self.w:])
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

        return beta

    def determine_following_distance(self, trust_score, ds, dacc):
        if trust_score > 0.8:
            return ds
        elif trust_score > 0.2:
            return ds + (dacc - ds) * (0.8 - trust_score)
        else:
            return dacc
