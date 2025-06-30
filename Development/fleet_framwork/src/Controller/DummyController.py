import numpy as np

# Createing the Dummy Controller with parameters for the real controller.
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
    def get_surrounding_vehicles(self, *args, **kwargs):
        return None, [dummy_leader], None, None  # You override this anyway
    
class DummyVehicle:
     def __init__(self, state, vehicle_id=0):
            self.state = state
            self.vehicle_id = vehicle_id
