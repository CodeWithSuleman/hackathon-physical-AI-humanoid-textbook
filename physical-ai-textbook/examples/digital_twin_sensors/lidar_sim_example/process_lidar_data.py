# examples/digital_twin_sensors/lidar_sim_example/process_lidar_data.py

import random
import math

def generate_dummy_lidar_scan(num_points=100, max_range=10.0):
    """
    Generates a conceptual 2D LiDAR scan with dummy data.
    Each point represents an angle and a distance.
    """
    scan_data = []
    for i in range(num_points):
        angle = (2 * math.pi / num_points) * i  # 0 to 2*pi radians
        distance = random.uniform(0.5, max_range) # Random distance within max_range
        # Simulate some "obstacles"
        if 2.0 < angle < 2.5 or 4.0 < angle < 4.5:
            distance = random.uniform(0.5, 3.0) # Closer obstacle
        scan_data.append({'angle_rad': angle, 'distance_m': distance})
    return scan_data

def process_lidar_scan(scan):
    """
    Processes a LiDAR scan, identifying minimum distance and simple obstacle detection.
    """
    if not scan:
        print("No scan data to process.")
        return

    min_distance = float('inf')
    obstacle_angles = []

    print("\n--- Processing LiDAR Scan ---")
    for point in scan:
        angle_deg = math.degrees(point['angle_rad'])
        distance = point['distance_m']
        
        # Simple obstacle detection threshold
        if distance < 1.0:
            obstacle_angles.append(f"{angle_deg:.2f}°")
        
        if distance < min_distance:
            min_distance = distance
        
        # print(f"Angle: {angle_deg:.2f}°, Distance: {distance:.2f}m")
    
    print(f"Minimum distance detected: {min_distance:.2f}m")
    if obstacle_angles:
        print(f"Obstacles detected at angles: {', '.join(obstacle_angles)}")
    else:
        print("No close obstacles detected.")
    print("-----------------------------\n")

if __name__ == "__main__":
    print("Generating dummy LiDAR scan data...")
    dummy_scan = generate_dummy_lidar_scan()
    process_lidar_scan(dummy_scan)
