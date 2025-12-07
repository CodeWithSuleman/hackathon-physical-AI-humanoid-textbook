# examples/digital_twin_sensors/imu_sim_example/process_imu_data.py

import random
import time

def generate_dummy_imu_data():
    """
    Generates conceptual IMU data (linear acceleration, angular velocity, orientation).
    For simplicity, this generates random data to simulate movement.
    """
    # Linear acceleration (m/s^2)
    accel_x = random.uniform(-1.0, 1.0)
    accel_y = random.uniform(-1.0, 1.0)
    accel_z = random.uniform(8.8, 10.8) # Simulate gravity + some noise
    
    # Angular velocity (rad/s)
    ang_vel_x = random.uniform(-0.5, 0.5)
    ang_vel_y = random.uniform(-0.5, 0.5)
    ang_vel_z = random.uniform(-0.5, 0.5)

    # Orientation (quaternion x, y, z, w - conceptual, not strictly valid quaternion math)
    # For a real IMU, this would be integrated from angular velocity or fused with magnetometer
    quat_x = random.uniform(-1.0, 1.0)
    quat_y = random.uniform(-1.0, 1.0)
    quat_z = random.uniform(-1.0, 1.0)
    quat_w = random.uniform(-1.0, 1.0)
    
    # Normalize quaternion conceptually
    magnitude = (quat_x**2 + quat_y**2 + quat_z**2 + quat_w**2)**0.5
    if magnitude > 0:
        quat_x /= magnitude
        quat_y /= magnitude
        quat_z /= magnitude
        quat_w /= magnitude

    return {
        'linear_acceleration': {'x': accel_x, 'y': accel_y, 'z': accel_z},
        'angular_velocity': {'x': ang_vel_x, 'y': ang_vel_y, 'z': ang_vel_z},
        'orientation': {'x': quat_x, 'y': quat_y, 'z': quat_z, 'w': quat_w},
        'timestamp': time.time()
    }

def process_imu_data(imu_data):
    """
    Processes IMU data, printing relevant information.
    """
    if not imu_data:
        print("No IMU data to process.")
        return

    print("\n--- Processing IMU Data ---")
    print(f"Timestamp: {imu_data['timestamp']:.2f}")
    print("Linear Acceleration (m/s^2):")
    print(f"  x: {imu_data['linear_acceleration']['x']:.2f}")
    print(f"  y: {imu_data['linear_acceleration']['y']:.2f}")
    print(f"  z: {imu_data['linear_acceleration']['z']:.2f}")
    print("Angular Velocity (rad/s):")
    print(f"  x: {imu_data['angular_velocity']['x']:.2f}")
    print(f"  y: {imu_data['angular_velocity']['y']:.2f}")
    print(f"  z: {imu_data['angular_velocity']['z']:.2f}")
    print("Orientation (Quaternion x,y,z,w):")
    print(f"  x: {imu_data['orientation']['x']:.2f}")
    print(f"  y: {imu_data['orientation']['y']:.2f}")
    print(f"  z: {imu_data['orientation']['z']:.2f}")
    print(f"  w: {imu_data['orientation']['w']:.2f}")
    print("---------------------------\n")

if __name__ == "__main__":
    print("Generating dummy IMU data...")
    for _ in range(3): # Simulate a few readings
        dummy_imu = generate_dummy_imu_data()
        process_imu_data(dummy_imu)
        time.sleep(0.1)
