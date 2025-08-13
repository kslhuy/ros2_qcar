#!/usr/bin/env python3
"""
Test script for Enhanced Vehicle State Management

This script demonstrates the improved state management system with:
- GPS time synchronization
- StateQueue for validated state management
- Time-based state validation
- State interpolation capabilities

Usage:
    python test_enhanced_vehicle.py [--verbose]
"""

import logging
import time
import threading
import argparse
from typing import Dict, Any

# Import the enhanced vehicle components
from StateQueue import StateQueue


class MockGPSSync:
    """Mock GPS sync for testing."""
    def __init__(self):
        self.offset = 0.0
        
    def get_synced_time(self):
        return time.time() + self.offset
        
    def sync_with_gps(self):
        # Simulate some time drift
        self.offset += 0.001  # 1ms drift


class MockVehicleState:
    """Mock vehicle state generator for testing."""
    def __init__(self, vehicle_id: int, start_pos: list = None):
        self.vehicle_id = vehicle_id
        self.pos = start_pos or [0, 0, 0]
        self.velocity = 1.0
        self.heading = 0.0
        self.seq = 0
        
    def generate_state(self, gps_sync, add_delay: float = 0.0) -> Dict[str, Any]:
        """Generate a mock vehicle state."""
        # Update position (simple forward motion)
        dt = 0.1  # 100ms
        self.pos[0] += self.velocity * dt
        
        # Create state with timestamp
        timestamp = gps_sync.get_synced_time() - add_delay
        
        state = {
            'type': 'state',
            'id': self.vehicle_id,
            'seq': self.seq,
            'pos': self.pos.copy(),
            'rot': [0, 0, self.heading],
            'v': self.velocity,
            'timestamp': timestamp
        }
        
        self.seq += 1
        return state


def test_state_queue_basic():
    """Test basic StateQueue functionality."""
    print("\n=== Testing Basic StateQueue Functionality ===")
    
    # Setup
    logger = logging.getLogger("StateQueueTest")
    gps_sync = MockGPSSync()
    
    state_queue = StateQueue(
        max_queue_size=10,
        max_age_seconds=1.0,
        max_delay_threshold=0.5,
        logger=logger
    )
    
    # Create mock vehicle states
    leader = MockVehicleState(vehicle_id=1, start_pos=[0, 0, 0])
    
    print("Adding valid states...")
    
    # Add some valid states
    for i in range(5):
        state = leader.generate_state(gps_sync)
        added = state_queue.add_state(state, gps_sync)
        print(f"  State {i}: {'Added' if added else 'Rejected'}")
        time.sleep(0.1)
    
    # Get latest state
    latest = state_queue.get_latest_valid_state(sender_id=1)
    print(f"Latest state: pos={latest['pos']}, seq={latest['seq']}")
    
    # Test old state rejection
    print("\nTesting old state rejection...")
    old_state = leader.generate_state(gps_sync, add_delay=2.0)  # 2 seconds old
    added = state_queue.add_state(old_state, gps_sync)
    print(f"Old state: {'Added' if added else 'Rejected (expected)'}")
    
    # Print statistics
    stats = state_queue.get_queue_stats()
    print(f"\nQueue Statistics:")
    print(f"  Total received: {stats['total_received']}")
    print(f"  Valid states: {stats['valid_states']}")
    print(f"  Expired states: {stats['expired_states']}")
    print(f"  Acceptance rate: {stats['acceptance_rate']:.1f}%")


def test_state_interpolation():
    """Test state interpolation functionality."""
    print("\n=== Testing State Interpolation ===")
    
    # Setup
    logger = logging.getLogger("InterpolationTest")
    gps_sync = MockGPSSync()
    
    state_queue = StateQueue(
        max_queue_size=20,
        max_age_seconds=2.0,
        max_delay_threshold=1.0,
        logger=logger
    )
    
    # Create mock vehicle with faster motion
    leader = MockVehicleState(vehicle_id=1, start_pos=[0, 0, 0])
    leader.velocity = 5.0  # 5 m/s for more noticeable interpolation
    
    print("Adding states for interpolation...")
    
    # Add states with some time spacing
    timestamps = []
    for i in range(4):
        state = leader.generate_state(gps_sync)
        state_queue.add_state(state, gps_sync)
        timestamps.append(state['timestamp'])
        print(f"  Added state {i}: pos=[{state['pos'][0]:.2f}, {state['pos'][1]:.2f}], "
              f"time={state['timestamp']:.3f}")
        time.sleep(0.2)  # 200ms between states
    
    # Test interpolation at different target times
    print("\nTesting interpolation:")
    
    # Interpolate between first and second state
    target_time = (timestamps[0] + timestamps[1]) / 2
    interpolated = state_queue.get_interpolated_state(target_time, sender_id=1)
    
    if interpolated:
        print(f"  Target time: {target_time:.3f}")
        print(f"  Interpolated pos: [{interpolated['pos'][0]:.2f}, {interpolated['pos'][1]:.2f}]")
        print(f"  Interpolated: {interpolated.get('interpolated', False)}")
        print(f"  Alpha: {interpolated.get('alpha', 0):.3f}")
    else:
        print("  Interpolation failed")
    
    # Test with target time outside range (should return nearest)
    future_time = timestamps[-1] + 0.5
    extrapolated = state_queue.get_interpolated_state(future_time, sender_id=1)
    
    if extrapolated:
        print(f"  Future target: {future_time:.3f}")
        print(f"  Returned pos: [{extrapolated['pos'][0]:.2f}, {extrapolated['pos'][1]:.2f}]")
        print(f"  (Should be latest available state)")


def test_communication_simulation():
    """Test communication with simulated network conditions."""
    print("\n=== Testing Communication Simulation ===")
    
    # Setup
    logger = logging.getLogger("CommTest")
    gps_sync = MockGPSSync()
    
    # Create state queues for leader and follower
    follower_queue = StateQueue(
        max_queue_size=30,
        max_age_seconds=1.0,
        max_delay_threshold=0.3,
        logger=logger
    )
    
    # Create mock vehicles
    leader = MockVehicleState(vehicle_id=1, start_pos=[0, 0, 0])
    
    print("Simulating communication with network delays and losses...")
    
    # Simulate communication with various conditions
    conditions = [
        {"name": "Normal", "delay": 0.02, "loss_rate": 0.0},  # 20ms delay, no loss
        {"name": "High Delay", "delay": 0.15, "loss_rate": 0.1},  # 150ms delay, 10% loss
        {"name": "Very High Delay", "delay": 0.4, "loss_rate": 0.0},  # 400ms delay (should be rejected)
        {"name": "Old Data", "delay": 1.5, "loss_rate": 0.0},  # 1.5s delay (should be rejected)
    ]
    
    for condition in conditions:
        print(f"\n  Testing {condition['name']} condition:")
        
        # Reset queue
        follower_queue.clear()
        
        # Send multiple states
        valid_count = 0
        total_sent = 10
        
        for i in range(total_sent):
            # Generate state
            state = leader.generate_state(gps_sync)
            
            # Simulate network delay
            state['timestamp'] -= condition['delay']
            
            # Simulate packet loss
            import random
            if random.random() < condition['loss_rate']:
                continue  # Packet lost
            
            # Try to add state
            if follower_queue.add_state(state, gps_sync):
                valid_count += 1
            
            time.sleep(0.05)  # 50ms between transmissions
        
        # Get statistics
        stats = follower_queue.get_queue_stats()
        print(f"    Valid states: {valid_count}/{total_sent}")
        print(f"    Queue size: {stats['current_queue_size']}")
        print(f"    Rejected (delay): {stats['delayed_states']}")
        print(f"    Rejected (expired): {stats['expired_states']}")


def main():
    """Main test function."""
    parser = argparse.ArgumentParser(description="Test Enhanced Vehicle State Management")
    parser.add_argument("--verbose", "-v", action="store_true", help="Enable verbose logging")
    
    args = parser.parse_args()
    
    # Setup logging
    level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    print("Enhanced Vehicle State Management Test Suite")
    print("=" * 50)
    
    try:
        # Run tests
        test_state_queue_basic()
        test_state_interpolation()
        test_communication_simulation()
        
        print("\n" + "=" * 50)
        print("All tests completed successfully!")
        
    except Exception as e:
        print(f"\nTest failed with error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == "__main__":
    exit(main())
