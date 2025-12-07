# Quickstart: Capstone Project: Full Autonomous Humanoid (ROS 2/Isaac)

This document provides a quick guide to setting up your environment for the Capstone Project and running the accompanying examples.

## Prerequisites

To follow along with the Capstone Project, you will need to have the following software installed and configured:

*   **NVIDIA GPU**: A compatible NVIDIA GPU with CUDA support is highly recommended for Isaac Sim and potentially for some Whisper or LLM inference setups.
*   **Linux Operating System**: Ubuntu 20.04 or 22.04 LTS is highly recommended for Isaac Sim, Isaac ROS, and ROS 2.
*   **Docker and NVIDIA Container Toolkit**: Required for running Isaac Sim, Isaac ROS, and potentially other ROS 2 components in a containerized environment.
*   **NVIDIA Isaac Sim**: Install Isaac Sim via Omniverse Launcher. Ensure you can launch it and access the Python API.
*   **NVIDIA Isaac ROS**: Follow the official Isaac ROS documentation for setting up the development environment, typically involving Docker containers and building workspaces.
*   **ROS 2 (Humble/Iron/Rolling)**: Ensure you have a working ROS 2 environment with Nav2 installed (from Module 3).
*   **Python 3.10+**: For running Whisper, LLM integration scripts, and ROS 2 nodes.
*   **OpenAI API Key**: Required for accessing OpenAI's Whisper API and potentially GPT models for LLM planning (if using remote API). For local LLMs, ensure the chosen model is downloaded and runnable.
*   **Microphone**: For voice input with Whisper.

## Running Whisper Voice-to-Command Demos

1.  **Set up Python Environment**: Ensure you have Python 3.10+ and necessary libraries (`openai`, `pydub`, `sounddevice` etc.) installed.
2.  **Navigate to Example**:
    ```bash
    cd examples/capstone_whisper_demos/
    ```
3.  **Run Demo Script**:
    ```bash
    python3 voice_to_command_demo.py
    ```
    (Specific commands will be provided with the examples.)

## Running LLM Cognitive Planning Demos

1.  **Set up Python Environment**: Ensure you have Python 3.10+ and libraries for LLM interaction (`openai`, `langchain` etc.) installed.
2.  **Configure API Key**: Set your `OPENAI_API_KEY` or equivalent environment variable.
3.  **Navigate to Example**:
    ```bash
    cd examples/capstone_llm_planning_demos/
    ```
4.  **Run Demo Script**:
    ```bash
    python3 nl_to_ros_action_demo.py
    ```
    (Specific commands will be provided with the examples.)

## Running Mini-Capstone Full VLA Demo

1.  **Ensure all prerequisites are met**: Isaac Sim, Isaac ROS, ROS 2, Nav2 are functional.
2.  **Launch Simulated Humanoid**: Start Isaac Sim with the humanoid robot and environment.
3.  **Launch ROS 2 Components**: Launch Isaac ROS, Nav2, and custom VLA nodes.
4.  **Issue Voice Command**: Use a microphone to issue a high-level command to the system.
5.  **Observe Autonomous Behavior**: Watch the simulated humanoid execute the task.

Specific instructions for each example will be provided within their respective example directories and detailed in the book chapters.
