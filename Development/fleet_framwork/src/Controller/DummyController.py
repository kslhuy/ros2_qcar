import numpy as np

# Creating the Dummy Controller with parameters for the real controller.
class DummyController:
    def __init__(self, qcar_id, config_params=None):
        """
        Initialize DummyController with configurable parameters.
        
        Args:
            qcar_id: ID of the QCar
            config_params: Dictionary of configuration parameters. If None, uses default values.
        """
        if config_params is None:
            # Default parameters (fallback)
            self.param_opt = {
                'alpha': 1.0,
                'beta': 1.5,
                'v0': 1.0,
                'delta': 4,
                'T': 0.4,
                's0': 7,  # for IDM
                'ri': 7,  # for CACC
                'hi': 0.5,
                'K': np.array([[1, 0.0], [0.0, 1]])  # for CACC
            }
        else:
            # Use provided configuration parameters
            self.param_opt = config_params.copy()
        
        self.param_sys = None
        self.goal = None
        self.straightlane = None
        self.vehicle_id = qcar_id
    
    def update_parameters(self, new_params):
        """Update controller parameters dynamically."""
        self.param_opt.update(new_params)
    
    def get_surrounding_vehicles(self, *args, **kwargs):
        # This method is typically overridden in the actual implementation
        return None, [], None, None
    
class DummyVehicle:
     def __init__(self, state, vehicle_id=0):
            self.state = state
            self.vehicle_id = vehicle_id
