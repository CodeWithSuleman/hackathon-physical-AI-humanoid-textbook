# Quickstart: Module 2: The Digital Twin (Gazebo & Unity)

This document provides a quick guide to setting up your environment for Module 2 and running the accompanying examples.

## Prerequisites

To follow along with Module 2, you will need to have the following software installed:

*   **ROS 2 Humble**: This is the base for our Gazebo and general robotics examples. Follow the official ROS 2 Humble installation guide for your operating system (Ubuntu recommended).
*   **Gazebo**: Gazebo is typically installed as part of your ROS 2 installation. Ensure you can launch Gazebo and interact with it.
*   **Unity Hub**: Download and install Unity Hub, then install a recent version of the Unity Editor (e.g., Unity 2022 LTS or newer).
*   **Unity Robotics Packages**: Within Unity, you will need to install specific packages for robotics integration, such as `ROS-TCP-Endpoint`, `ROS-TCP-Connector`, and potentially `URDF-Importer`. Instructions for this will be detailed in the book content.
*   **Python 3.10+**: For running ROS 2 Python examples and any helper scripts.

## Running Gazebo Examples

1.  **Source your ROS 2 environment**:
    ```bash
    source /opt/ros/humble/setup.bash
    ```
2.  **Navigate to the Gazebo example directory**:
    ```bash
    cd examples/digital_twin_gazebo_world/
    ```
3.  **Launch the Gazebo world**:
    ```bash
    ros2 launch digital_twin_gazebo_world launch_world.launch.py
    ```
    (Specific launch commands will be provided with the examples.)

## Running Unity Examples

1.  **Open Unity Hub**: Launch Unity Hub and add the `examples/digital_twin_unity_scene/` project.
2.  **Open the Unity Project**: Open the project in the Unity Editor.
3.  **Navigate to the example scene**: Open the relevant scene file (e.g., `Assets/Scenes/HumanoidSensorScene.unity`).
4.  **Run the scene**: Press the Play button in the Unity Editor to start the simulation and visualization.

## Sensor Simulation Examples

Specific instructions for running LiDAR, Depth Camera, and IMU sensor simulation examples will be provided within their respective example directories and detailed in the book chapters. This will typically involve running a Unity scene or a Gazebo simulation.
