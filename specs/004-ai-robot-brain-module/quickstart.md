# Quickstart: Module 3: The AI-Robot Brain (NVIDIA Isaac™)

This document provides a quick guide to setting up your environment for Module 3 and running the accompanying examples.

## Prerequisites

To follow along with Module 3, you will need to have the following software installed and configured:

*   **NVIDIA GPU**: A compatible NVIDIA GPU with CUDA support is required for NVIDIA Isaac Sim and Isaac ROS.
*   **Linux Operating System**: Ubuntu 20.04 or 22.04 LTS is highly recommended for NVIDIA Isaac Sim and Isaac ROS.
*   **Docker and NVIDIA Container Toolkit**: Required for running Isaac Sim and Isaac ROS containers.
*   **NVIDIA Isaac Sim**: Install Isaac Sim via Omniverse Launcher. Ensure you can launch it and access the Python API.
*   **NVIDIA Isaac ROS**: Follow the official Isaac ROS documentation for setting up the development environment, typically involving Docker containers and building workspaces.
*   **ROS 2 (Humble/Iron/Rolling)**: Isaac ROS is built on ROS 2. Ensure you have a working ROS 2 environment.
*   **Nav2**: Nav2 is usually installed as part of your ROS 2 distribution or through dedicated packages.
*   **Python 3.10+**: For running Isaac Sim scripts, Isaac ROS nodes, and any helper scripts.

## Running Isaac Sim Examples

1.  **Launch Isaac Sim**: Start Isaac Sim from Omniverse Launcher.
2.  **Open example scene**: Navigate to the example scene within Isaac Sim (specific path will be provided with the example).
3.  **Execute Python scripts**: Run Python scripts (e.g., from the Isaac Sim Script Editor or a separate terminal connected to the Isaac Sim environment) to control simulation, generate synthetic data, or interact with robots.

## Running Isaac ROS VSLAM + Navigation Examples

1.  **Set up Isaac ROS Workspace**: Follow Isaac ROS documentation to build and source your workspace in a Docker container or native environment.
2.  **Launch VSLAM pipeline**:
    ```bash
    ros2 launch isaac_ros_visual_slam visual_slam.launch.py # (Example launch command)
    ```
3.  **Launch Nav2 stack**:
    ```bash
    ros2 launch nav2_bringup bringup_launch.py # (Example launch command)
    ```
    (Specific launch commands and setup will be provided with the examples.)

## Running Nav2 Path Planning Examples

1.  **Launch a simulated robot with Nav2**: This typically involves a ROS 2 launch file that brings up a robot in a Gazebo or Isaac Sim environment, along with the Nav2 stack.
2.  **Set a navigation goal**: Use RViz2 or a programmatic interface to set a 2D navigation goal for the robot.
3.  **Observe path planning**: Observe the robot generate a path and execute it in the simulation.

Specific instructions for each example will be provided within their respective example directories and detailed in the book chapters.
