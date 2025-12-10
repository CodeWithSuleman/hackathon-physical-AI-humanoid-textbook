# examples/capstone_llm_planning_demos/nl_to_ros_action_demo/nl_to_ros_action_demo.py

import os
import random
import json
import time
from dotenv import load_dotenv

# Load environment variables for conceptual OpenAI API Key
load_dotenv()
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")

# Conceptual LLM interaction (no actual API call made here)
def conceptual_llm_generate(prompt):
    """
    Simulates an LLM generating a ROS 2 action sequence from a natural language prompt.
    """
    print(f"\n--- Simulating LLM Request ---")
    print(f"LLM Prompt (partial): {prompt[:100]}...")
    time.sleep(1) # Simulate LLM thinking time

    # Dummy responses based on keywords for demonstration
    prompt_lower = prompt.lower()
    
    if "go to kitchen" in prompt_lower and "fetch cup" in prompt_lower:
        actions = [
            {"action_type": "navigate", "target": "kitchen"},
            {"action_type": "detect_object", "object_type": "cup", "location": "kitchen"},
            {"action_type": "grasp_object", "object_id": "cup_01"},
            {"action_type": "navigate", "target": "table"},
            {"action_type": "place_object", "target": "table_center"}
        ]
        return json.dumps(actions, indent=2)
    elif "move to door" in prompt_lower:
        actions = [
            {"action_type": "navigate", "target": "door"},
            {"action_type": "wait", "duration_seconds": 2}
        ]
        return json.dumps(actions, indent=2)
    elif "find nearest person" in prompt_lower:
        actions = [
            {"action_type": "detect_person", "strategy": "nearest"},
            {"action_type": "approach_target", "target_type": "person", "max_distance": 1.0}
        ]
        return json.dumps(actions, indent=2)
    else:
        return json.dumps([{"action_type": "log_message", "message": f"Could not plan for: {prompt_lower}"}])

def validate_ros2_action_sequence(action_sequence_json):
    """
    Conceptual function to validate if the generated JSON is a valid ROS 2 action sequence.
    In a real system, this would check against action definitions.
    """
    try:
        actions = json.loads(action_sequence_json)
        if not isinstance(actions, list):
            return False, "Not a list of actions."
        for action in actions:
            if not isinstance(action, dict) or "action_type" not in action:
                return False, "Each action must be a dictionary with 'action_type'."
        return True, "Valid conceptual action sequence."
    except json.JSONDecodeError:
        return False, "Invalid JSON format."

if __name__ == "__main__":
    print("--- LLM Cognitive Planning Demo: NL to ROS 2 Action Sequence ---")

    if not OPENAI_API_KEY:
        print("Warning: OPENAI_API_KEY not set. Using conceptual LLM interaction without actual API calls.")
        print("To run with actual OpenAI models (e.g., for LangChain integration), set OPENAI_API_KEY.")

    while True:
        natural_language_task = input("\nEnter a natural language task for the robot (e.g., 'go to kitchen and fetch the cup', or 'exit' to quit): \n> ")
        
        if natural_language_task.lower() == 'exit':
            break

        # Simulate LLM generating a plan
        llm_response_json = conceptual_llm_generate(natural_language_task)
        
        print("\n--- LLM Generated Plan (JSON) ---")
        print(llm_response_json)
        
        # Conceptual validation
        is_valid, validation_msg = validate_ros2_action_sequence(llm_response_json)
        print(f"\nConceptual Validation: {validation_msg} (Is Valid: {is_valid})")

        # Further processing in a real system would involve dispatching these actions
        print("\n--- End of Plan ---")

    print("\nDemo finished.")
