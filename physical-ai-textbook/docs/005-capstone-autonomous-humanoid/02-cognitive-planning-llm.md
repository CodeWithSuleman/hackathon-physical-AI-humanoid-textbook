# Cognitive Planning with LLMs: Natural Language to ROS 2 Action Sequence

Building on our ability to accurately transcribe voice commands with Whisper, this chapter dives into the "brain" of our autonomous humanoid: a Large Language Model (LLM) for cognitive planning. We will explore how LLMs can interpret high-level natural language tasks and translate them into executable sequences of ROS 2 actions, bridging the gap between human intent and robot behavior.

## The Role of LLMs in Cognitive Planning

Traditional robot planning often relies on predefined state machines or symbolic planners, which can be rigid and difficult to adapt to novel situations or nuanced natural language commands. Large Language Models (LLMs) offer a powerful alternative by providing:

*   **Semantic Understanding**: Interpreting the meaning and context of natural language instructions.
*   **Reasoning**: Inferring sub-goals and logical steps required to achieve a high-level task.
*   **World Knowledge**: Leveraging vast amounts of pre-trained knowledge to inform planning decisions (though this needs to be constrained to the robot's capabilities and environment).
*   **Flexibility**: Adapting to variations in phrasing and unforeseen situations more gracefully than hard-coded logic.

In the VLA pipeline, the LLM acts as the central planner, taking the transcribed command and generating a plan that the robot can execute.

## Translating Natural Language to ROS 2 Action Sequences

The core challenge in using LLMs for cognitive planning in robotics is transforming the LLM's natural language output into a structured, executable format that the robot's ROS 2 system can understand. This involves:

1.  **Prompt Engineering**: Crafting effective prompts for the LLM that:
    *   Clearly define the robot's capabilities (available ROS 2 actions).
    *   Provide context about the robot's environment.
    *   Instruct the LLM on the desired output format (e.g., a JSON list of actions).
    *   Specify constraints or safety guidelines.
2.  **LLM Inference**: Sending the natural language task, along with the engineered prompt, to the LLM (either a remote API like OpenAI's GPT models or a locally deployed LLM).
3.  **Output Parsing**: Parsing the LLM's response to extract the structured sequence of ROS 2 actions. This often involves robust JSON parsing or pattern matching.
4.  **Action Dispatch**: Executing the parsed ROS 2 actions sequentially, monitoring their status, and potentially providing feedback to the LLM for re-planning if actions fail.

### Defining ROS 2 Actions

ROS 2 Actions provide a goal-feedback-result interaction pattern suitable for long-running tasks like navigation or manipulation. Each action has:

*   **Goal**: The request sent to the action server (e.g., `NavigateToGoal` with a target pose).
*   **Feedback**: Intermediate updates on the progress towards the goal (e.g., current pose during navigation).
*   **Result**: The final outcome of the action (e.g., `success`, `failure`).

The LLM's plan needs to map directly to these defined ROS 2 action interfaces.

## Running the LLM Cognitive Planning Demo

You can find a conceptual example demonstrating LLM cognitive planning in `examples/capstone_llm_planning_demos/nl_to_ros_action_demo/`. Refer to the `README.md` file within that directory for detailed setup and execution instructions.

To run this demo:

1.  **Ensure Prerequisites**: Make sure you have Python 3.10+ and your `OPENAI_API_KEY` environment variable is set (or a local LLM is configured).
2.  **Install Python Libraries**: The script requires `langchain` (and `openai` if using OpenAI models). Install them using `pip install -r requirements.txt` from the project root.
3.  **Navigate to Example**:
    ```bash
    cd examples/capstone_llm_planning_demos/nl_to_ros_action_demo/
    ```
4.  **Run the Script**:
    ```bash
    python3 nl_to_ros_action_demo.py
    ```
    The script will prompt you for a natural language task and simulate an LLM generating a conceptual ROS 2 action sequence.

## Conclusion

Large Language Models are revolutionizing cognitive planning in robotics by enabling robots to understand and execute tasks from high-level natural language commands. By carefully crafting prompts and effectively parsing LLM outputs into ROS 2 action sequences, we empower humanoid robots with flexible, intelligent decision-making capabilities. This forms the central "brain" of our autonomous system, which we will integrate with voice commands and physical execution in our mini-capstone project.

## Further Reading

*   [OpenAI API Documentation](https://platform.openai.com/docs/api-reference)
*   [LangChain Documentation](https://python.langchain.com/docs/get_started/introduction)
*   [ROS 2 Actions Conceptual Overview](https://docs.ros.org/en/rolling/Concepts/Basic/About-Actions.html)
*   [VLA Research Papers (Google Scholar)](https://scholar.google.com/scholar?q=vision+language+action+robotics)
