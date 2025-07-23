# Configuration Examples for Fleet Simulation
# Copy and modify these examples in your main.py

from FleetConfig import FleetConfig, ConfigPresets, RoadType, ControllerType

# Example 1: Simple preset usage
def get_openroad_cacc_config():
    """OpenRoad environment with CACC controller."""
    return ConfigPresets.get_openroad_cacc()

def get_studio_idm_config():
    """Studio environment with IDM controller."""
    return ConfigPresets.get_studio_idm()

# Example 2: Custom configuration
def get_custom_config():
    """Custom configuration with specific parameters."""
    config = FleetConfig(RoadType.OpenRoad, ControllerType.CACC)
    
    # Modify specific parameters
    config.update_config(
        simulation_time=60,
        qcar_num=3,
        distance_between_cars=0.3,
        s0=5,  # Closer following distance
        ri=5,  # Closer CACC distance
        max_velocity=1.5,
        lookahead_distance=5.0
    )
    
    return config

# Example 3: Complex node sequence
def get_complex_path_config():
    """Configuration with complex path."""
    config = ConfigPresets.get_openroad_cacc()
    config.update_config(
        node_sequence=[10, 4, 20, 13, 10],
        simulation_time=120,
        qcar_num=4
    )
    return config

# Example 4: Aggressive following configuration
def get_aggressive_following_config():
    """Configuration for aggressive following behavior."""
    config = ConfigPresets.get_studio_cacc()
    config.update_config(
        s0=2,  # Very close following
        ri=2,  # Very close CACC
        T=0.2,  # Fast response
        distance_between_cars=0.1,
        max_velocity=0.8
    )
    return config

# Example 5: Conservative following configuration
def get_conservative_following_config():
    """Configuration for conservative following behavior."""
    config = ConfigPresets.get_openroad_idm()
    config.update_config(
        s0=10,  # Large following distance
        ri=10,  # Large CACC distance
        T=0.8,  # Slow response
        distance_between_cars=0.5,
        max_velocity=1.0
    )
    return config

# How to use in main.py:
"""
# Option 1: Use preset
config = ConfigPresets.get_openroad_cacc()

# Option 2: Use custom function
config = get_custom_config()

# Option 3: Create and modify on the fly
config = FleetConfig(RoadType.Studio, ControllerType.IDM)
config.update_config(simulation_time=30, qcar_num=2)

# Option 4: Load based on command line argument or environment
import sys
if len(sys.argv) > 1:
    if sys.argv[1] == "studio":
        config = ConfigPresets.get_studio_cacc()
    elif sys.argv[1] == "aggressive":
        config = get_aggressive_following_config()
    else:
        config = ConfigPresets.get_openroad_cacc()
else:
    config = ConfigPresets.get_openroad_cacc()
"""
