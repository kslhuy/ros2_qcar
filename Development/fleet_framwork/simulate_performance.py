#!/usr/bin/env python3
"""
Simulate CommHandler performance monitoring without starting the full fleet
"""

import time
import json
from performance_monitor import perf_monitor

def simulate_message_processing():
    """Simulate what happens when CommHandler processes messages"""
    
    print("=== SIMULATING COMMHANDLER MESSAGE PROCESSING ===\n")
    
    # Simulate receiving and processing 10 messages
    for i in range(10):
        print(f"Processing message {i+1}/10...")
        
        # Simulate JSON decoding (like in _process_received_message)
        json_start = perf_monitor.start_timing()
        
        # Simulate JSON decode work
        test_message = {
            'type': 'state',
            'seq': i,
            'id': 1,
            'pos': [1.0, 2.0, 3.0],
            'rot': [0.0, 0.0, 0.0],
            'v': 5.5,
            'timestamp': time.time(),
            'ack_port': 12346
        }
        json_data = json.dumps(test_message)
        parsed_message = json.loads(json_data)
        time.sleep(0.002)  # Simulate 2ms JSON work
        
        perf_monitor.end_timing(json_start, "json_decode")
        
        # Simulate overall processing (like in _handle_state_message_optimized)
        process_start = perf_monitor.start_timing()
        
        # Simulate validation work
        validation_start = perf_monitor.start_timing()
        time.sleep(0.003)  # Simulate 3ms validation work
        perf_monitor.end_timing(validation_start, "validation")
        
        # Simulate some processing work
        time.sleep(0.045)  # Simulate 45ms processing work
        
        perf_monitor.end_timing(process_start, "processing")
        
        time.sleep(0.1)  # Brief pause between messages
    
    print("\n=== PERFORMANCE RESULTS ===")
    perf_monitor.print_performance_report()

if __name__ == "__main__":
    simulate_message_processing()
