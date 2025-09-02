"""
Integration example showing how to use the new data logging system 
with your existing VehicleObserver code.
"""

import numpy as np
from BasicDataLogger import BasicObserverDataLogger

def example_integration_with_existing_observer():
    """
    Example of how to integrate the data logger with your existing observer system.
    This shows the minimal changes needed to add data logging.
    """
    
    print("=== Fleet Observer Data Logging Integration Example ===\n")
    
    # Your existing simulation parameters
    fleet_size = 2
    simulation_time = 5.0  # seconds
    dt = 0.01  # 100 Hz
    
    # Create data loggers for each vehicle
    data_loggers = {}
    for vehicle_id in range(fleet_size):
        data_loggers[vehicle_id] = BasicObserverDataLogger(
            vehicle_id=vehicle_id, 
            fleet_size=fleet_size,
            log_dir="integration_demo_logs"
        )
    
    print(f"Created data loggers for {fleet_size} vehicles\n")
    
    # Simulation loop (simplified version of your existing code)
    n_steps = int(simulation_time / dt)
    
    for step in range(n_steps):
        timestamp = step * dt
        
        for vehicle_id in range(fleet_size):
            # === YOUR EXISTING OBSERVER CODE ===
            # (This is where your current observer.update_local_state() call would be)
            
            # Simulate getting data from your observer
            # Replace these with your actual observer outputs:
            estimated_local_state = simulate_observer_local_state(vehicle_id, timestamp)
            measured_gps_state = simulate_gps_measurement(vehicle_id, timestamp, step)
            control_input = simulate_control_input(vehicle_id, timestamp)
            fleet_estimates = simulate_fleet_estimates(vehicle_id, fleet_size, timestamp)
            
            # === NEW: ADD DATA LOGGING ===
            # Just add these two lines to your existing code:
            
            # 1. Log local state data
            data_loggers[vehicle_id].log_local_state(
                timestamp=timestamp,
                local_state=estimated_local_state,
                measured_state=measured_gps_state,
                control_input=control_input,
                gps_available=measured_gps_state is not None,
                dt=dt
            )
            
            # 2. Log fleet state estimates
            data_loggers[vehicle_id].log_fleet_states(
                timestamp=timestamp,
                fleet_states=fleet_estimates
            )
    
    print(f"Simulation completed after {n_steps} steps")
    
    # === NEW: SAVE DATA AT END OF SIMULATION ===
    print("\n=== Saving Data ===")
    saved_directories = []
    
    for vehicle_id in range(fleet_size):
        print(f"Saving data for Vehicle {vehicle_id}...")
        data_loggers[vehicle_id].close()  # This automatically saves CSV, JSON, and MATLAB files
        saved_directories.append(data_loggers[vehicle_id].data_dir)
    
    # Show user what was created
    print(f"\n=== Data Export Completed ===")
    print(f"Data saved in: {saved_directories[0]}")
    print("\nGenerated files for each vehicle:")
    print("📄 local_state_vehicle_N.csv    - Real-time local state data")
    print("📄 fleet_states_vehicle_N.csv   - Real-time fleet estimates") 
    print("📄 observer_data_vehicle_N.json - Structured data")
    print("📄 observer_data_vehicle_N.mat  - MATLAB format")
    
    print(f"\n=== How to Use the Data ===")
    print("Option 1 - Python Analysis:")
    print("  import pandas as pd")
    print(f"  data = pd.read_csv('{saved_directories[0]}/local_state_vehicle_0.csv')")
    print("  # Analyze with pandas, numpy, matplotlib")
    
    print("\nOption 2 - MATLAB Analysis:")
    print("  % In MATLAB:")
    for vehicle_id in range(fleet_size):
        mat_file = f"observer_data_vehicle_{vehicle_id}_*.mat"
        print(f"  plot_observer_data('{saved_directories[0]}/{mat_file}')")
    
    return saved_directories[0]


def simulate_observer_local_state(vehicle_id: int, timestamp: float) -> np.ndarray:
    """Simulate your observer's local state output."""
    # Replace this with: observer.get_local_state() or similar
    
    # Simple circular motion simulation
    radius = 3.0 + vehicle_id * 1.0
    omega = 0.5  # rad/s
    x = radius * np.cos(omega * timestamp)
    y = radius * np.sin(omega * timestamp)
    theta = omega * timestamp + np.pi/2
    velocity = radius * omega
    
    return np.array([x, y, theta, velocity])


def simulate_gps_measurement(vehicle_id: int, timestamp: float, step: int) -> np.ndarray:
    """Simulate GPS measurements (available intermittently)."""
    # Replace this with: your actual GPS data or None if not available
    
    # GPS available every 20 steps (simulating 5 Hz GPS)
    if step % 20 == 0:
        true_state = simulate_observer_local_state(vehicle_id, timestamp)
        # Add GPS noise
        gps_noise = np.array([0.1, 0.1, 0.02, 0.05])  # [x, y, theta, v] noise
        return true_state + np.random.normal(0, gps_noise)
    else:
        return None


def simulate_control_input(vehicle_id: int, timestamp: float) -> np.ndarray:
    """Simulate control inputs."""
    # Replace this with: your actual control inputs
    
    steering = 0.1 * np.sin(0.5 * timestamp)
    acceleration = 0.2 * np.cos(0.3 * timestamp)
    return np.array([steering, acceleration])


def simulate_fleet_estimates(vehicle_id: int, fleet_size: int, timestamp: float) -> np.ndarray:
    """Simulate fleet state estimates."""
    # Replace this with: observer.get_fleet_states() or similar
    
    fleet_states = np.zeros((4, fleet_size))
    for i in range(fleet_size):
        fleet_states[:, i] = simulate_observer_local_state(i, timestamp)
    
    # Add some estimation noise
    fleet_states += np.random.normal(0, 0.05, fleet_states.shape)
    
    return fleet_states


def analyze_saved_data(data_directory: str):
    """Show how to analyze the saved data."""
    print(f"\n=== Analyzing Saved Data from {data_directory} ===")
    
    import os
    import glob
    
    # Find all CSV files
    csv_files = glob.glob(os.path.join(data_directory, "*.csv"))
    mat_files = glob.glob(os.path.join(data_directory, "*.mat"))
    
    print(f"Found {len(csv_files)} CSV files and {len(mat_files)} MATLAB files")
    
    # Try to load and analyze one CSV file
    try:
        import pandas as pd
        
        local_csv = [f for f in csv_files if 'local_state' in f]
        if local_csv:
            print(f"\nAnalyzing: {local_csv[0]}")
            df = pd.read_csv(local_csv[0])
            
            print(f"Data shape: {df.shape}")
            print(f"Time range: {df['timestamp'].min():.3f} to {df['timestamp'].max():.3f} seconds")
            print(f"GPS availability: {df['gps_available'].mean()*100:.1f}%")
            print(f"Position range: X=[{df['x'].min():.2f}, {df['x'].max():.2f}], Y=[{df['y'].min():.2f}, {df['y'].max():.2f}]")
            
            print("\nFirst few rows:")
            print(df.head(3).to_string())
    
    except ImportError:
        print("pandas not available, skipping CSV analysis")
    
    # Show MATLAB file info
    if mat_files:
        try:
            from scipy.io import loadmat
            
            print(f"\nMATLAB file structure: {mat_files[0]}")
            mat_data = loadmat(mat_files[0])
            
            # Show available data fields
            data_fields = [key for key in mat_data.keys() if not key.startswith('__')]
            print(f"Available data fields: {data_fields}")
            
            if 'metadata' in mat_data:
                metadata = mat_data['metadata']
                print(f"Metadata: {metadata}")
        
        except ImportError:
            print("scipy not available, skipping MATLAB analysis")


if __name__ == "__main__":
    # Run the integration example
    data_dir = example_integration_with_existing_observer()
    
    # Analyze the results
    analyze_saved_data(data_dir)
    
    print(f"\n{'='*60}")
    print("🎉 Integration example completed successfully!")
    print("\nTo integrate with your existing code:")
    print("1. Add: from BasicDataLogger import BasicObserverDataLogger")
    print("2. Create: logger = BasicObserverDataLogger(vehicle_id, fleet_size)")
    print("3. In your simulation loop, add:")
    print("   - logger.log_local_state(timestamp, local_state, gps_state, control, gps_available, dt)")
    print("   - logger.log_fleet_states(timestamp, fleet_estimates)")
    print("4. At end: logger.close()")
    print("\nThat's it! Your data will be automatically saved in multiple formats.")
