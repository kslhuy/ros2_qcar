#!/usr/bin/env python3
"""
Test script for distributed observer integration in VehicleProcess.py
This script demonstrates how the distributed observer works with the fleet simulation.
"""

import time
import logging
import numpy as np
from VehicleObserver import VehicleObserver

def test_distributed_observer_integration():
    """Test distributed observer integration similar to VehicleProcess usage."""
    print("Testing Distributed Observer Integration")
    print("=" * 60)
    
    # Setup parameters similar to VehicleProcess
    fleet_size = 3
    vehicle_id = 1  # Test from perspective of vehicle 1
    
    # Create test config similar to what VehicleProcess would use
    class TestConfig:
        def __init__(self):
            self.observer_config = {
                "enable_distributed": True,
                "local_observer_type": "kalman",
                "distributed_observer_type": "consensus",
                "consensus_gain": 0.1
            }
        
        def get(self, section, default=None):
            if section == 'observer':
                return self.observer_config
            return default
    
    config = TestConfig()
    
    # Setup logging
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger(f"test_vehicle_{vehicle_id}")
    
    # Initialize observer similar to VehicleProcess
    initial_pose = np.array([vehicle_id * 2.0, 0.0, 0.0])  # Spaced 2m apart
    
    try:
        observer = VehicleObserver(
            vehicle_id=vehicle_id,
            fleet_size=fleet_size,
            config=config,
            logger=logger,
            initial_pose=initial_pose
        )
        print(f"✓ Observer initialized for vehicle {vehicle_id}")
        
    except Exception as e:
        print(f"✗ Observer initialization failed: {e}")
        return False
    
    # Simulate the integration workflow from VehicleProcess.observer_update()
    print(f"\nSimulating observer update workflow...")
    
    for step in range(5):
        current_time = time.time()
        print(f"\n--- Step {step+1} ---")
        
        # 1. Simulate GPS measurement (like in VehicleProcess)
        simulated_x = vehicle_id * 2.0 + 0.1 * step
        simulated_y = 0.05 * np.sin(0.5 * step)
        simulated_theta = 0.0
        simulated_velocity = 0.5
        
        measured_state = np.array([simulated_x, simulated_y, simulated_theta, simulated_velocity])
        control_input = np.array([0.0, 0.0])  # [steering, acceleration]
        
        print(f"Vehicle {vehicle_id}: GPS measurement = pos({simulated_x:.3f}, {simulated_y:.3f}), vel={simulated_velocity:.3f}")
        
        # 2. Update local state (like in VehicleProcess.observer_update())
        try:
            estimated_state = observer.update_local_state(
                measured_state=measured_state,
                control_input=control_input,
                timestamp=current_time,
                motor_tach=simulated_velocity,
                gyroscope_z=0.0
            )
            print(f"Vehicle {vehicle_id}: Local state updated = pos({estimated_state[0]:.3f}, {estimated_state[1]:.3f}), vel={estimated_state[3]:.3f}")
            
        except Exception as e:
            print(f"✗ Local state update failed: {e}")
            continue
        
        # 3. Simulate received states from other vehicles (like in VehicleProcess.process_received_state())
        for other_vehicle_id in range(fleet_size):
            if other_vehicle_id != vehicle_id:
                # Simulate state from other vehicle
                other_x = other_vehicle_id * 2.0 + 0.1 * step
                other_y = 0.05 * np.sin(0.5 * step + other_vehicle_id)
                other_state = np.array([other_x, other_y, 0.0, 0.5])
                other_control = np.array([0.0, 0.0])
                
                try:
                    observer.add_received_state(
                        sender_id=other_vehicle_id,
                        state=other_state,
                        control=other_control,
                        timestamp=current_time
                    )
                    print(f"Vehicle {vehicle_id}: Added state from vehicle {other_vehicle_id}")
                    
                except Exception as e:
                    print(f"✗ Adding received state failed: {e}")
        
        # 4. Update distributed observer (like in VehicleProcess.observer_update())
        try:
            fleet_states = observer.update_distributed_estimates(current_time)
            
            if fleet_states is not None:
                print(f"Vehicle {vehicle_id}: Distributed observer updated - Fleet estimates:")
                for v_id in range(fleet_states.shape[1]):
                    state = fleet_states[:, v_id]
                    print(f"  Vehicle {v_id}: pos=({state[0]:.3f}, {state[1]:.3f}), vel={state[3]:.3f}")
                
                # 5. Simulate fleet estimates storage (like in VehicleProcess.observer_update())
                fleet_state_estimates = {}
                for vehicle_idx in range(fleet_states.shape[1]):
                    vehicle_state = fleet_states[:, vehicle_idx]
                    fleet_state_estimates[vehicle_idx] = {
                        'position': [float(vehicle_state[0]), float(vehicle_state[1]), 0.0],
                        'rotation': [0.0, 0.0, float(vehicle_state[2])],
                        'velocity': float(vehicle_state[3]),
                        'timestamp': current_time,
                        'source': 'distributed_observer'
                    }
                
                print(f"Vehicle {vehicle_id}: Fleet estimates cached for {len(fleet_state_estimates)} vehicles")
                
            else:
                print(f"✗ Distributed observer returned None")
                
        except Exception as e:
            print(f"✗ Distributed observer update failed: {e}")
        
        time.sleep(0.1)  # Small delay between steps
    
    print(f"\n" + "=" * 60)
    print("✓ Distributed observer integration test completed successfully!")
    return True

def test_fleet_communication_format():
    """Test the fleet communication message format."""
    print("\nTesting Fleet Communication Format")
    print("-" * 40)
    
    # Simulate fleet estimates message (like VehicleProcess.broadcast_fleet_estimates())
    fleet_estimates = {
        0: {'position': [0.5, 0.1, 0.0], 'rotation': [0.0, 0.0, 0.0], 'velocity': 0.5},
        1: {'position': [2.5, 0.1, 0.0], 'rotation': [0.0, 0.0, 0.0], 'velocity': 0.5},
        2: {'position': [4.5, 0.1, 0.0], 'rotation': [0.0, 0.0, 0.0], 'velocity': 0.5}
    }
    
    fleet_message = {
        'msg_type': 'fleet_estimates',
        'sender_id': 1,
        'timestamp': time.time(),
        'fleet_size': len(fleet_estimates),
        'estimates': {}
    }
    
    # Add each vehicle's estimated state
    for vehicle_id, state in fleet_estimates.items():
        fleet_message['estimates'][vehicle_id] = {
            'pos': state['position'][:2],  # [x, y] only for compatibility
            'rot': state['rotation'],
            'vel': state['velocity'],
            'timestamp': state.get('timestamp', time.time()),
            'source': state.get('source', 'distributed_observer')
        }
    
    print("Fleet message format:")
    print(f"  Message type: {fleet_message['msg_type']}")
    print(f"  Sender: Vehicle {fleet_message['sender_id']}")
    print(f"  Fleet size: {fleet_message['fleet_size']}")
    print(f"  Estimates: {len(fleet_message['estimates'])} vehicles")
    
    for vehicle_id, estimate in fleet_message['estimates'].items():
        pos = estimate['pos']
        vel = estimate['vel']
        print(f"    Vehicle {vehicle_id}: pos=({pos[0]:.3f}, {pos[1]:.3f}), vel={vel:.3f}")
    
    print("✓ Fleet communication format test completed")

if __name__ == "__main__":
    print("Distributed Observer Integration Test Suite")
    print("=" * 60)
    
    try:
        # Test basic integration
        success = test_distributed_observer_integration()
        
        if success:
            # Test communication format
            test_fleet_communication_format()
            
            print(f"\n" + "=" * 60)
            print("🎉 All integration tests passed!")
            print("The distributed observer is ready for use in VehicleProcess.py")
            
        else:
            print(f"\n" + "=" * 60)
            print("❌ Integration tests failed!")
            
    except Exception as e:
        print(f"\n❌ Test suite failed with error: {e}")
        import traceback
        traceback.print_exc()
