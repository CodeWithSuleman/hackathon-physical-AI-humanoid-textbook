# LLM Cognitive Planning Demo (Conceptual)

This directory is intended to host a conceptual example demonstrating how a Large Language Model (LLM) might translate natural language tasks into ROS 2 action sequences. Due to the need for LLM access (API key or local model setup), this example provides a conceptual Python script to simulate the process.

## Conceptual Setup

A real LLM cognitive planning demo would involve:
*   **LLM Access**: Configured access to an LLM (e.g., OpenAI's API via `OPENAI_API_KEY`, or a locally running LLM).
*   **Prompt Engineering**: Carefully designed prompts to instruct the LLM on available robot actions and desired output format.
*   **ROS 2 Action Definitions**: Defined ROS 2 actions that the robot can execute.

## Generating ROS 2 Action Sequences (Conceptual Python Script)

The `nl_to_ros_action_demo.py` script included here demonstrates how one might provide a natural language task to an LLM and receive a conceptual ROS 2 action sequence. It uses dummy data and a simulated LLM interaction to illustrate the process.

### Running the Python Script

1.  Navigate to this directory:
    ```bash
    cd examples/capstone_llm_planning_demos/nl_to_ros_action_demo/
    ```
2.  Run the script:
    ```bash
    python3 nl_to_ros_action_demo.py
    ```
    This script will prompt you for a natural language task and then print a conceptual ROS 2 action sequence.

## Further Exploration

To set up and run a full LLM cognitive planning example, refer to:
*   [OpenAI API Documentation](https://platform.openai.com/docs/api-reference)
*   [LangChain Documentation](https://python.langchain.com/docs/get_started/introduction)
*   [ROS 2 Actions Conceptual Overview](https://docs.ros.org/en/humble/Concepts/Basic/Concepts-Actions.html)
