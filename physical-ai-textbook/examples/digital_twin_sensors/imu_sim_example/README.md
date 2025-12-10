# IMU Simulation Example (Conceptual)

This directory is intended to host an example demonstrating IMU (Inertial Measurement Unit) sensor simulation. Due to the complexity of setting up a full-fledged IMU simulation (which typically involves Gazebo plugins or Unity scripts reading physics data), this example provides a conceptual Python script to process IMU-like data.

## Conceptual Setup

A real IMU simulation would involve:
*   **In Gazebo**: Integrating an `imu` sensor into a robot's SDF/URDF model, which then publishes `sensor_msgs/Imu` messages.
*   **In Unity**: Attaching a custom script to a robot GameObject that reads `Rigidbody` physics data (linear/angular velocities, accelerations) and formats it as IMU readings.

## Processing Simulated IMU Data (Conceptual Python Script)

The `process_imu_data.py` script included here demonstrates how one might process simulated IMU data. It does not connect to a live simulation but works with dummy data.

### Running the Python Script

1.  Navigate to this directory:
    ```bash
    cd examples/digital_twin_sensors/imu_sim_example/
    ```
2.  Run the script:
    ```bash
    python3 process_imu_data.py
    ```
    This script will print conceptual IMU readings (e.g., acceleration, angular velocity) to the console.

## Further Exploration

To set up a full IMU simulation, refer to the official documentation for:
*   [Gazebo Sensors (IMU)](https://gazebosim.org/docs/latest/sensors#imu-sensor)
*   [Unity Robotics Sensors](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/6_sensors.md)
