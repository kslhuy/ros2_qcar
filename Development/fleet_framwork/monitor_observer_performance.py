#!/usr/bin/env python3
"""
Observer Performance Monitor

This script monitors the observer performance by analyzing log files 
and providing real-time timing analysis for the local observer updates.

Usage:
    python monitor_observer_performance.py [vehicle_id]
    
Example:
    python monitor_observer_performance.py 1
    # Monitors observer performance for Vehicle 1
"""

import time
import os
import re
from collections import deque, defaultdict
from typing import Dict, List, Optional
import argparse
import threading

class ObserverPerformanceMonitor:
    """Monitor observer performance from log files."""
    
    def __init__(self, vehicle_id: int, log_dir: str = "logs"):
        self.vehicle_id = vehicle_id
        self.log_dir = log_dir
        self.log_file = os.path.join(log_dir, f"observer_vehicle_{vehicle_id}.log")
        
        # Timing statistics storage - separated by source
        self.timing_history = deque(maxlen=1000)  # Keep last 1000 measurements
        self.timing_stats = {
            'vehicle': {
                'total': deque(maxlen=100),
                'gps': deque(maxlen=100), 
                'sensor': deque(maxlen=100),
                'obs_update': deque(maxlen=100),
                'control_state': deque(maxlen=100),
                'state_update': deque(maxlen=100),
                'distributed': deque(maxlen=100)
            },
            'observer': {
                'total': deque(maxlen=100),
                'dt_calc': deque(maxlen=100),
                'state_update': deque(maxlen=100),
                'gps_track': deque(maxlen=100),
                'fleet_update': deque(maxlen=100),
                'logging': deque(maxlen=100)
            },
            'ekf': {
                'total': deque(maxlen=100),
                'control_prep': deque(maxlen=100),
                'ekf_update': deque(maxlen=100),
                'state_extract': deque(maxlen=100)
            },
            'distributed': {
                'total': deque(maxlen=100),
                'max_vehicle': deque(maxlen=100),
                'avg_vehicle': deque(maxlen=100),
                'state_copy': deque(maxlen=100)
            }
        }
        
        # State tracking
        self.gps_availability = deque(maxlen=100)
        self.ekf_status = deque(maxlen=100)
        self.state_sources = deque(maxlen=100)
        
        # Performance alerts
        self.performance_warnings = []
        self.last_file_position = 0
        
        print(f"Observer Performance Monitor for Vehicle {vehicle_id}")
        print(f"Monitoring log file: {self.log_file}")
        print("-" * 60)
    
    def parse_timing_line(self, line: str) -> Optional[Dict]:
        """Parse a timing log line and extract measurements."""
        # Look for Vehicle.py TIMING lines  
        vehicle_timing_match = re.search(r'TIMING: Total=(\d+\.?\d*)ms, GPS=(\d+\.?\d*)ms, Sensor=(\d+\.?\d*)ms, ObsUpdate=(\d+\.?\d*)ms, ControlState=(\d+\.?\d*)ms, StateUpdate=(\d+\.?\d*)ms, Distributed=(\d+\.?\d*)ms', line)
        
        if vehicle_timing_match:
            return {
                'timestamp': self._extract_timestamp(line),
                'source': 'vehicle',
                'total': float(vehicle_timing_match.group(1)),
                'gps': float(vehicle_timing_match.group(2)),
                'sensor': float(vehicle_timing_match.group(3)),
                'obs_update': float(vehicle_timing_match.group(4)),
                'control_state': float(vehicle_timing_match.group(5)),
                'state_update': float(vehicle_timing_match.group(6)),
                'distributed': float(vehicle_timing_match.group(7))
            }
        
        # Look for VehicleObserver.py OBS_TIMING lines
        obs_timing_match = re.search(r'OBS_TIMING: Total=(\d+\.?\d*)ms, DTCalc=(\d+\.?\d*)ms, StateUpdate=(\d+\.?\d*)ms, GPSTrack=(\d+\.?\d*)ms, FleetUpdate=(\d+\.?\d*)ms, Logging=(\d+\.?\d*)ms', line)
        
        if obs_timing_match:
            return {
                'timestamp': self._extract_timestamp(line),
                'source': 'observer',
                'total': float(obs_timing_match.group(1)),
                'dt_calc': float(obs_timing_match.group(2)),
                'state_update': float(obs_timing_match.group(3)),
                'gps_track': float(obs_timing_match.group(4)),
                'fleet_update': float(obs_timing_match.group(5)),
                'logging': float(obs_timing_match.group(6))
            }
        
        # Look for EKF_TIMING lines
        ekf_timing_match = re.search(r'EKF_TIMING: Total=(\d+\.?\d*)ms, ControlPrep=(\d+\.?\d*)ms, EKFUpdate=(\d+\.?\d*)ms, StateExtract=(\d+\.?\d*)ms', line)
        
        if ekf_timing_match:
            return {
                'timestamp': self._extract_timestamp(line),
                'source': 'ekf',
                'total': float(ekf_timing_match.group(1)),
                'control_prep': float(ekf_timing_match.group(2)),
                'ekf_update': float(ekf_timing_match.group(3)),
                'state_extract': float(ekf_timing_match.group(4))
            }
        
        # Look for DIST_TIMING lines
        dist_timing_match = re.search(r'DIST_TIMING: Total=(\d+\.?\d*)ms, MaxVehicle=(\d+\.?\d*)ms, AvgVehicle=(\d+\.?\d*)ms, StateCopy=(\d+\.?\d*)ms, Vehicles=(\d+)', line)
        
        if dist_timing_match:
            return {
                'timestamp': self._extract_timestamp(line),
                'source': 'distributed',
                'total': float(dist_timing_match.group(1)),
                'max_vehicle': float(dist_timing_match.group(2)),
                'avg_vehicle': float(dist_timing_match.group(3)),
                'state_copy': float(dist_timing_match.group(4)),
                'vehicles': int(dist_timing_match.group(5))
            }
        
        return None
    
    def parse_state_line(self, line: str) -> Optional[Dict]:
        """Parse a state log line and extract information."""
        # Look for STATE lines
        state_match = re.search(r'STATE: GPS_avail=(\w+), EKF_init=(\w+), Source=(\w+)', line)
        
        if state_match:
            return {
                'timestamp': self._extract_timestamp(line),
                'gps_available': state_match.group(1) == 'True',
                'ekf_initialized': state_match.group(2) == 'True',
                'source': state_match.group(3)
            }
        return None
    
    def parse_performance_warning(self, line: str) -> Optional[Dict]:
        """Parse performance warning lines."""
        if 'PERFORMANCE:' in line and 'slow' in line:
            perf_match = re.search(r'Observer update slow \((\d+\.?\d*)ms\)', line)
            if perf_match:
                return {
                    'timestamp': self._extract_timestamp(line),
                    'timing': float(perf_match.group(1)),
                    'type': 'slow_update'
                }
        return None
    
    def _extract_timestamp(self, line: str) -> str:
        """Extract timestamp from log line."""
        timestamp_match = re.match(r'(\d{2}:\d{2}:\d{2}\.\d{3})', line)
        return timestamp_match.group(1) if timestamp_match else "unknown"
    
    def update_statistics(self, timing_data: Dict):
        """Update timing statistics with new data."""
        source = timing_data.get('source', 'vehicle')  # Default to vehicle for backward compatibility
        
        if source in self.timing_stats:
            for key, value in timing_data.items():
                if key not in ['timestamp', 'source'] and key in self.timing_stats[source]:
                    self.timing_stats[source][key].append(value)
        
        self.timing_history.append(timing_data)
    
    def get_current_stats(self) -> Dict:
        """Get current performance statistics."""
        if not any(any(values for values in source_stats.values()) for source_stats in self.timing_stats.values()):
            return {'error': 'No timing data available yet'}
        
        stats = {}
        for source, source_stats in self.timing_stats.items():
            stats[source] = {}
            for component, values in source_stats.items():
                if values:
                    stats[source][component] = {
                        'current': values[-1],
                        'avg': sum(values) / len(values),
                        'min': min(values),
                        'max': max(values),
                        'count': len(values)
                    }
        
        # Add GPS and EKF status
        if self.gps_availability:
            gps_available_count = sum(1 for x in self.gps_availability if x)
            stats['gps_availability_rate'] = gps_available_count / len(self.gps_availability) * 100
        
        if self.ekf_status:
            ekf_init_count = sum(1 for x in self.ekf_status if x)
            stats['ekf_initialization_rate'] = ekf_init_count / len(self.ekf_status) * 100
        
        return stats
    
    def print_current_stats(self):
        """Print current performance statistics to console."""
        stats = self.get_current_stats()
        
        if 'error' in stats:
            print(f"No data available yet...")
            return
        
        print(f"\n=== Vehicle {self.vehicle_id} Observer Performance (Last {len(self.timing_history)} updates) ===")
        
        # Print timing statistics by source
        for source, source_stats in stats.items():
            if source in ['gps_availability_rate', 'ekf_initialization_rate']:
                continue  # Skip these for now
                
            if isinstance(source_stats, dict) and source_stats:
                print(f"\n--- {source.upper()} TIMING ---")
                for component, data in source_stats.items():
                    if isinstance(data, dict) and 'current' in data:
                        print(f"{component.upper():15} | Current: {data['current']:6.2f}ms | "
                              f"Avg: {data['avg']:6.2f}ms | Min: {data['min']:6.2f}ms | "
                              f"Max: {data['max']:6.2f}ms")
        
        # Print status rates
        if 'gps_availability_rate' in stats:
            print(f"\nGPS Availability: {stats['gps_availability_rate']:.1f}%")
        if 'ekf_initialization_rate' in stats:
            print(f"EKF Initialized:  {stats['ekf_initialization_rate']:.1f}%")
        
        # Performance warnings - check vehicle total time
        vehicle_total = None
        if 'vehicle' in stats and 'total' in stats['vehicle']:
            vehicle_total = stats['vehicle']['total'].get('avg', 0)
        
        if vehicle_total and vehicle_total > 10.0:
            print(f"⚠️  WARNING: Average observer update time ({vehicle_total:.2f}ms) > 10ms target")
        elif vehicle_total and vehicle_total > 5.0:
            print(f"⚡ CAUTION: Average observer update time ({vehicle_total:.2f}ms) > 5ms")
        else:
            print(f"✅ GOOD: Observer performance within targets")
        
        print("-" * 80)
    
    def monitor_log_file(self):
        """Monitor the log file for new entries."""
        if not os.path.exists(self.log_file):
            print(f"Log file {self.log_file} not found. Waiting...")
            while not os.path.exists(self.log_file):
                time.sleep(1)
            print(f"Log file found. Starting monitoring...")
        
        with open(self.log_file, 'r') as f:
            # Move to last known position
            f.seek(self.last_file_position)
            
            while True:
                line = f.readline()
                if not line:
                    # No new data, sleep and try again
                    time.sleep(0.1)
                    continue
                
                # Update file position
                self.last_file_position = f.tell()
                
                # Parse timing data
                timing_data = self.parse_timing_line(line)
                if timing_data:
                    self.update_statistics(timing_data)
                
                # Parse state data
                state_data = self.parse_state_line(line)
                if state_data:
                    self.gps_availability.append(state_data['gps_available'])
                    self.ekf_status.append(state_data['ekf_initialized'])
                    self.state_sources.append(state_data['source'])
                
                # Parse performance warnings
                warning_data = self.parse_performance_warning(line)
                if warning_data:
                    self.performance_warnings.append(warning_data)
                    print(f"⚠️  PERFORMANCE WARNING: {warning_data}")
    
    def run_monitor(self, display_interval: float = 2.0):
        """Run the performance monitor with periodic display updates."""
        # Start log monitoring in background thread
        monitor_thread = threading.Thread(target=self.monitor_log_file, daemon=True)
        monitor_thread.start()
        
        print("Observer Performance Monitor started. Press Ctrl+C to exit.\n")
        
        try:
            while True:
                self.print_current_stats()
                time.sleep(display_interval)
        except KeyboardInterrupt:
            print("\nMonitor stopped.")

def main():
    parser = argparse.ArgumentParser(description='Monitor Observer Performance')
    parser.add_argument('vehicle_id', type=int, nargs='?', default=1,
                      help='Vehicle ID to monitor (default: 1)')
    parser.add_argument('--log-dir', type=str, default='logs',
                      help='Log directory path (default: logs)')
    parser.add_argument('--interval', type=float, default=2.0,
                      help='Display update interval in seconds (default: 2.0)')
    
    args = parser.parse_args()
    
    monitor = ObserverPerformanceMonitor(args.vehicle_id, args.log_dir)
    monitor.run_monitor(args.interval)

if __name__ == "__main__":
    main()
