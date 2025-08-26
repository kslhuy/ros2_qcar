#!/usr/bin/env python3
"""
PAL GPS Performance Test

This script tests the performance of QCarGPS.readGPS() calls
using the PAL framework (pal.products.qcar) to compare with
the QLabs framework performance.

This will help determine if GPS slowness is specific to QLabs
or a general issue with the GPS simulation.
"""

import time
import statistics
from typing import List
import os


from qvl.qlabs import QuanserInteractiveLabs
from qvl.qcar2 import QLabsQCar2
from qvl.real_time import QLabsRealTime
from hal.products.mats import SDCSRoadMap
from hal.content.qcar_functions import QCarEKF



# PAL Framework imports
try:
    from pal.products.qcar import QCar, QCarGPS, IS_PHYSICAL_QCAR
    PAL_AVAILABLE = True
except ImportError as e:
    print(f"Warning: PAL framework not available: {e}")
    PAL_AVAILABLE = False

def test_pal_gps_performance():
    """Test PAL QCarGPS performance."""
    
    if not PAL_AVAILABLE:
        print("Cannot run test - PAL framework not available")
        return
    
    print("=== PAL QCarGPS Performance Test ===\n")
    
    # Configuration
    calibrationPose = [0, 2, -1.5708]  # [x, y, theta] - pi/2
    test_duration = 15  # seconds
    target_rate = 100  # Hz
    
    print(f"Testing QCarGPS.readGPS() for {test_duration} seconds at {target_rate}Hz")
    print("Calibration pose:", calibrationPose)
    print()

    qlabs = QuanserInteractiveLabs()
    qlabs.open("localhost")

    qlabs.destroy_all_spawned_actors()
    QLabsRealTime().terminate_all_real_time_models()

    


    leader = QLabsQCar2(qlabs)
    leader.spawn_id(actorNumber=0, location=[-1.205, -0.83, 0.005], rotation=[0, 0, -44.7], scale=[0.1, 0.1, 0.1])

    rtModel = os.path.normpath(os.path.join(os.environ['RTMODELS_DIR'], 'QCar2/QCar2_Workspace_studio'))
    QLabsRealTime().start_real_time_model(rtModel, actorNumber=0)

    roadmap = SDCSRoadMap(leftHandTraffic=False)

    nodeSequence = [10, 4, 20, 13, 10]
    waypointSequence = roadmap.generate_path(nodeSequence)
    initialPose = roadmap.get_node_pose(nodeSequence[0]).squeeze()
    print("Initial Pose:", initialPose)

    time.sleep(1)



    # Initialize PAL components
    try:
        qcar = QCar(readMode=1, frequency=target_rate)
        ekf = QCarEKF(x_0=initialPose)
        gps = QCarGPS(initialPose=calibrationPose, calibrate=False)
        
        print("PAL QCar and GPS initialized successfully\n")
        
    except Exception as e:
        print(f"Failed to initialize PAL components: {e}")
        return
    
    # Test GPS performance
    gps_times = []
    successful_reads = 0
    failed_reads = 0
    loop_times = []
    
    with qcar, gps:
        print("Starting GPS performance test...")
        print("\nDetailed GPS Call Results:")
        print("Call#  | Time (ms) | Success | Position (x, y, z)      | Orientation (r, p, y)   | Status")
        print("-------|-----------|---------|-------------------------|-------------------------|-------")
        
        start_time = time.time()
        loop_count = 0
        
        while time.time() - start_time < test_duration:
            loop_start = time.perf_counter()
            
            # Read QCar data (for context)
            qcar.read()
            
            # Test GPS read timing
            gps_start = time.perf_counter()
            gps_success = gps.readGPS()
            gps_end = time.perf_counter()
            
            gps_time_ms = (gps_end - gps_start) * 1000
            gps_times.append(gps_time_ms)
            
            if gps_success:
                successful_reads += 1
                # Get GPS data for verification
                pos = gps.position
                orientation = gps.orientation
                
                # Determine status
                status = "SLOW" if gps_time_ms > 8.0 else "OK" if gps_time_ms > 5.0 else "FAST"
                success_str = "YES"
                
                # Print every 50 calls for detailed view, or every 10 for first 100 calls
                if loop_count < 100 and loop_count % 10 == 0:
                    print(f"{loop_count+1:5d}  | {gps_time_ms:7.3f}   | {success_str:7s} | ({pos[0]:6.3f}, {pos[1]:6.3f}, {pos[2]:6.3f}) | ({orientation[0]:6.3f}, {orientation[1]:6.3f}, {orientation[2]:6.3f}) | {status}")
                elif loop_count >= 100 and loop_count % 50 == 0:
                    print(f"{loop_count+1:5d}  | {gps_time_ms:7.3f}   | {success_str:7s} | ({pos[0]:6.3f}, {pos[1]:6.3f}, {pos[2]:6.3f}) | ({orientation[0]:6.3f}, {orientation[1]:6.3f}, {orientation[2]:6.3f}) | {status}")
            else:
                failed_reads += 1
                status = "FAIL"
                success_str = "NO"
                if loop_count % 50 == 0:
                    print(f"{loop_count+1:5d}  | {gps_time_ms:7.3f}   | {success_str:7s} | N/A                     | N/A                     | {status}")
            
            loop_end = time.perf_counter()
            total_loop_time = (loop_end - loop_start) * 1000
            loop_times.append(total_loop_time)
            
            loop_count += 1
            
            # Maintain target rate
            target_interval = 1.0 / target_rate
            elapsed = time.time() - (start_time + loop_count * target_interval)
            if elapsed < target_interval:
                time.sleep(target_interval - elapsed)
    
    # Analyze results
    if gps_times:
        print(f"\n=== Results ===")
        print(f"Test duration: {time.time() - start_time:.1f}s")
        print(f"Total loops: {loop_count}")
        print(f"Successful GPS reads: {successful_reads}")
        print(f"Failed GPS reads: {failed_reads}")
        print(f"GPS success rate: {successful_reads/(successful_reads+failed_reads)*100:.1f}%")
        
        # GPS timing statistics
        avg_gps = statistics.mean(gps_times)
        min_gps = min(gps_times)
        max_gps = max(gps_times)
        std_gps = statistics.stdev(gps_times) if len(gps_times) > 1 else 0
        
        print(f"\nGPS Performance:")
        print(f"  Average GPS time: {avg_gps:.3f}ms")
        print(f"  Min GPS time: {min_gps:.3f}ms")
        print(f"  Max GPS time: {max_gps:.3f}ms")
        print(f"  Std deviation: {std_gps:.3f}ms")
        
        # Categorize GPS call times
        fast_calls = sum(1 for t in gps_times if t < 2.0)
        moderate_calls = sum(1 for t in gps_times if 2.0 <= t < 5.0)
        slow_calls = sum(1 for t in gps_times if 5.0 <= t < 10.0)
        very_slow_calls = sum(1 for t in gps_times if t >= 10.0)
        
        print(f"\nGPS Call Distribution:")
        print(f"  Fast (<2ms): {fast_calls} ({fast_calls/len(gps_times)*100:.1f}%)")
        print(f"  Moderate (2-5ms): {moderate_calls} ({moderate_calls/len(gps_times)*100:.1f}%)")
        print(f"  Slow (5-10ms): {slow_calls} ({slow_calls/len(gps_times)*100:.1f}%)")
        print(f"  Very slow (>10ms): {very_slow_calls} ({very_slow_calls/len(gps_times)*100:.1f}%)")
        
        # Loop timing statistics
        if loop_times:
            avg_loop = statistics.mean(loop_times)
            target_loop_time = 1000.0 / target_rate
            
            print(f"\nOverall Loop Performance:")
            print(f"  Average loop time: {avg_loop:.3f}ms (Target: {target_loop_time:.1f}ms)")
            print(f"  Actual rate achieved: {1000.0/avg_loop:.1f}Hz (Target: {target_rate}Hz)")
        
        # Performance analysis
        print(f"\n=== Analysis ===")
        if avg_gps > 8.0:
            print("🔴 GPS BOTTLENECK: PAL GPS calls are very slow (>8ms average)")
            print("   This confirms GPS is a significant performance bottleneck")
        elif avg_gps > 5.0:
            print("🟡 GPS MODERATE: PAL GPS calls are somewhat slow (>5ms average)")
            print("   GPS contributes to performance issues")
        else:
            print("✅ GPS GOOD: PAL GPS calls are fast (<5ms average)")
            print("   GPS is not a significant bottleneck")
        
        # Comparison with target
        target_loop_time = 1000.0 / target_rate
        gps_percentage = (avg_gps / target_loop_time) * 100
        
        print(f"\nGPS Impact on {target_rate}Hz loop:")
        print(f"  GPS uses {gps_percentage:.1f}% of available loop time")
        print(f"  Remaining time for other operations: {target_loop_time - avg_gps:.1f}ms")
        
        if gps_percentage > 80:
            print("❌ CRITICAL: GPS uses most of the loop time")
        elif gps_percentage > 50:
            print("⚠️  WARNING: GPS uses significant portion of loop time")
        else:
            print("✅ ACCEPTABLE: GPS leaves sufficient time for other operations")
        
        # Comparison with QLabs expectations
        print(f"\n=== PAL vs QLabs Comparison ===")
        print(f"PAL GPS Average: {avg_gps:.3f}ms")
        print(f"Expected QLabs GPS: ~8-10ms (from previous analysis)")
        
        if avg_gps < 3.0:
            print("🟢 PAL ADVANTAGE: PAL GPS significantly faster than QLabs")
            print("   → Bottleneck is QLabs-specific, not GPS simulation in general")
        elif avg_gps < 6.0:
            print("🟡 PAL BETTER: PAL GPS moderately faster than QLabs")
            print("   → Some GPS overhead, but QLabs adds additional delay")
        else:
            print("🔴 SIMILAR BOTTLENECK: PAL GPS also slow")
            print("   → GPS simulation itself is the fundamental bottleneck")
        
        print(f"\nRecommendations based on PAL performance:")
        if avg_gps < 3.0:
            print("  • Consider switching from QLabs to PAL framework for better performance")
            print("  • If stuck with QLabs, reduce GPS rate to 25-50Hz")
        elif avg_gps < 6.0:
            print("  • PAL framework offers better performance but still has GPS overhead")
            print("  • For QLabs: reduce GPS rate to 50-75Hz")
        else:
            print("  • GPS simulation is fundamentally slow in both frameworks")
            print("  • Reduce GPS rates significantly in any framework")

def compare_with_target_rates():
    """Test GPS performance at different target rates."""
    
    if not PAL_AVAILABLE:
        return
        
    print("\n=== GPS Performance at Different Rates ===")
    
    test_rates = [50, 100, 200]  # Hz
    calibrationPose = [0, 2, -1.5708]
    test_duration = 10  # seconds per rate
    
    for rate in test_rates:
        print(f"\nTesting at {rate}Hz for {test_duration}s...")
        
        try:
            qcar = QCar(readMode=1, frequency=rate)
            gps = QCarGPS(initialPose=calibrationPose, calibrate=False)
            
            gps_times = []
            
            with qcar, gps:
                start_time = time.time()
                loop_count = 0
                
                while time.time() - start_time < test_duration:
                    qcar.read()
                    
                    gps_start = time.perf_counter()
                    gps.readGPS()
                    gps_time = (time.perf_counter() - gps_start) * 1000
                    
                    gps_times.append(gps_time)
                    loop_count += 1
                    
                    # Maintain rate
                    target_interval = 1.0 / rate
                    elapsed = time.time() - (start_time + loop_count * target_interval)
                    if elapsed < target_interval:
                        time.sleep(target_interval - elapsed)
            
            if gps_times:
                avg_gps = statistics.mean(gps_times)
                target_loop = 1000.0 / rate
                gps_percent = (avg_gps / target_loop) * 100
                
                print(f"  {rate}Hz: GPS avg={avg_gps:.3f}ms ({gps_percent:.1f}% of {target_loop:.1f}ms loop)")
                
        except Exception as e:
            print(f"  {rate}Hz: Failed - {e}")

if __name__ == "__main__":
    try:
        test_pal_gps_performance()
        # compare_with_target_rates()
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Test failed: {e}")
        import traceback
        traceback.print_exc()
