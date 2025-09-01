#!/usr/bin/env python3
"""
Test script for distributed observer implementation matching MATLAB Observer.m
"""

import numpy as np
import sys
import os
import time
import logging

# Add the current directory to Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from VehicleObserver import VehicleObserver

class TestConfig:
    """Simple config class for testing"""
    def __init__(self):
        self.observer_config = {
            "local_observer_type": "kalman",
            "enable_distributed": True,
            "distributed_observer_type": "consensus",
            "enable_noise_measurement": False,
            "enable_prediction": True,
            "consensus_gain": 0.1
        }
    
    def get(self, section, default=None):
        if section == 'observer':
            return self.observer_config
        return default

def test_distributed_observer():
    """Test the distributed observer functionality."""
    print("Testing Distributed Observer Implementation")
    print("=" * 50)
    
    # Setup test parameters
    fleet_size = 3
    test_duration = 5.0  # seconds
    dt = 0.1  # 10 Hz update rate
    
    # Create test config
    config = TestConfig()
    
    # Setup logging
    logging.basicConfig(level=logging.INFO)
    test_logger = logging.getLogger(f"test_observer")
    
    # Create observers for each vehicle
    observers = []
    for vehicle_id in range(fleet_size):
        initial_pose = {
            'position': [vehicle_id * 2.0, 0.0, 0.0],  # Spaced 2m apart
            'rotation': [0.0, 0.0, 0.0]
        }
        
        observer = VehicleObserver(
            vehicle_id=vehicle_id,
            fleet_size=fleet_size,
            config=config,
            logger=test_logger,  # Provide logger
            initial_pose=initial_pose
        )
        observers.append(observer)
        print(f"Created observer for vehicle {vehicle_id}")
    
    print(f"\nRunning distributed observer test for {test_duration} seconds...")
    
    # Test simulation loop
    start_time = time.time()
    step = 0
    
    while (time.time() - start_time) < test_duration:
        current_time = time.time()
        timestamp = current_time - start_time
        
        # Simulate GPS updates for each vehicle
        for i, observer in enumerate(observers):
            # Simulate vehicle movement (each moves forward at different speeds)
            x = i * 2.0 + 0.5 * timestamp  # Starting position + movement
            y = 0.0 + 0.1 * np.sin(2 * np.pi * 0.1 * timestamp)  # Small oscillation
            theta = 0.0  # Heading
            velocity = 0.5  # m/s
            
            # Create measured state for local update
            measured_state = np.array([x, y, theta, velocity])
            control_input = np.array([0.0, 0.0])  # [steering, acceleration]
            
            # Update local state with GPS measurement
            observer.update_local_state(
                measured_state=measured_state, 
                control_input=control_input, 
                timestamp=timestamp,
                motor_tach=velocity,  # Use velocity as motor tach
                gyroscope_z=0.0      # Provide gyroscope reading
            )
            
            # Send state to other observers (simulate communication)
            for j, other_observer in enumerate(observers):
                if i != j:
                    other_observer.add_received_state(
                        sender_id=i,
                        state=measured_state,
                        control=control_input,
                        timestamp=timestamp
                    )
        
        # Update distributed estimates for all observers
        for i, observer in enumerate(observers):
            try:
                fleet_estimates = observer.update_distributed_estimates(timestamp)
                
                if step % 10 == 0:  # Print every 1 second
                    print(f"\nStep {step}: Vehicle {i} Fleet Estimates:")
                    for vehicle_id in range(fleet_size):
                        state = fleet_estimates[:, vehicle_id]
                        print(f"  Vehicle {vehicle_id}: pos=({state[0]:.3f},{state[1]:.3f}), "
                              f"theta={state[2]:.3f}, vel={state[3]:.3f}")
                
            except Exception as e:
                print(f"Error in distributed observer for vehicle {i}: {e}")
                import traceback
                traceback.print_exc()
        
        step += 1
        time.sleep(dt)
    
    print(f"\nTest completed after {step} steps")
    print("=" * 50)

def test_communication_weights():
    """Test the communication weights matrix generation."""
    print("\nTesting Communication Weights Matrix")
    print("-" * 40)
    
    fleet_size = 3
    config = TestConfig()
    
    observer = VehicleObserver(
        vehicle_id=0,
        fleet_size=fleet_size,
        config=config,
        logger=None
    )
    
    weights = observer._get_distributed_weights()
    print(f"Weights matrix for fleet size {fleet_size}:")
    print(weights)
    
    # Verify properties
    print(f"\nMatrix properties:")
    print(f"- Shape: {weights.shape}")
    print(f"- Diagonal (should be zeros): {np.diag(weights)}")
    print(f"- Row sums (should be 1.0): {np.sum(weights, axis=1)}")
    
    # Test fully connected property
    off_diagonal = weights[np.eye(fleet_size) == 0]
    print(f"- All off-diagonal elements > 0: {np.all(off_diagonal > 0)}")

def test_system_matrices():
    """Test system dynamics and observation matrices."""
    print("\nTesting System Matrices")
    print("-" * 30)
    
    config = TestConfig()
    observer = VehicleObserver(
        vehicle_id=0,
        fleet_size=2,
        config=config,
        logger=None
    )
    
    # Test dynamics matrix
    A = observer._get_system_dynamics_matrix()
    print(f"System dynamics matrix A:")
    print(A)
    print(f"Shape: {A.shape}")
    
    # Test observation matrix
    C = observer._get_observation_matrix()
    print(f"\nObservation matrix C:")
    print(C)
    print(f"Shape: {C.shape}")
    
    # Test observer gain
    L = observer._get_observer_gain(0)
    print(f"\nObserver gain matrix L:")
    print(L)
    print(f"Shape: {L.shape}")

if __name__ == "__main__":
    print("Distributed Observer Test Suite")
    print("Based on MATLAB Observer.m implementation")
    print("=" * 60)
    
    try:
        # Test individual components
        test_communication_weights()
        test_system_matrices()
        
        # Test full distributed observer
        test_distributed_observer()
        
        print("\n" + "=" * 60)
        print("All tests completed successfully!")
        
    except Exception as e:
        print(f"\nTest failed with error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
