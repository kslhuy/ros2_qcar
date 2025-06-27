import numpy as np

class Comm:
    def __init__(self, host_controller, vehicle_dict):
        self.controller = host_controller
        self.vehicles = vehicle_dict

    def get_local_state(self, id, host_id):
        if id in self.vehicles:
            return self.vehicles[id].state
        return np.zeros(4)

    def get_global_state(self, id, host_id):
        return self.controller.observer.est_global_state_current

    def get_input(self, id):
        if id in self.vehicles:
            return self.vehicles[id].input
        return np.zeros(1)
