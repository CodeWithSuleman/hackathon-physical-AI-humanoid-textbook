# examples/capstone_full_vla_demo/mini_capstone_vla_demo/mini_capstone_vla_demo.py

import random
import time
import json

# --- Conceptual Components (Simulated) ---

def simulate_whisper_transcription(audio_input):
    """Simulates OpenAI Whisper transcribing audio to text."""
    print(f"\n[Whisper] Processing audio input: '{audio_input}'...")
    time.sleep(1)
    if "go to kitchen and fetch the cup" in audio_input.lower():
        return "go to kitchen and fetch the cup"
    elif "find the red block and place it on the table" in audio_input.lower():
        return "find the red block and place it on the table"
    elif "move to the charging station" in audio_input.lower():
        return "move to the charging station"
    else:
        return f"unrecognized command: {audio_input}"

def simulate_llm_planning(natural_language_task):
    """Simulates LLM generating a ROS 2 action sequence from natural language."""
    print(f"[LLM Planner] Receiving task: '{natural_language_task}'")
    time.sleep(2)
    
    if "go to kitchen and fetch the cup" in natural_language_task.lower():
        actions = [
            {"action_type": "navigate", "target": "kitchen"},
            {"action_type": "detect_object", "object_type": "cup", "location": "kitchen"},
            {"action_type": "grasp_object", "object_id": "cup_01"},
            {"action_type": "navigate", "target": "delivery_point"},
            {"action_type": "place_object", "target": "delivery_point"}
        ]
    elif "find the red block and place it on the table" in natural_language_task.lower():
        actions = [
            {"action_type": "navigate", "target": "search_area"},
            {"action_type": "detect_object", "object_type": "red block"},
            {"action_type": "grasp_object", "object_id": "red_block_01"},
            {"action_type": "navigate", "target": "table"},
            {"action_type": "place_object", "target": "table_surface"}
        ]
    elif "move to the charging station" in natural_language_task.lower():
        actions = [
            {"action_type": "navigate", "target": "charging_station"},
            {"action_type": "dock_robot"}
        ]
    else:
        actions = [{"action_type": "log_message", "message": f"LLM could not plan for: {natural_language_task}"}]
    
    print(f"[LLM Planner] Generated plan:\n{json.dumps(actions, indent=2)}")
    return actions

def simulate_ros2_action_execution(action):
    """Simulates a ROS 2 action server executing a single action."""
    print(f"[ROS 2 Action Executor] Executing action: {action['action_type']} with params {action.get('target', action.get('object_type', ''))}...")
    time.sleep(random.uniform(1, 3)) # Simulate action duration
    success = random.choice([True, True, True, False]) # Simulate occasional failure
    if success:
        print(f"[ROS 2 Action Executor] Action '{action['action_type']}' completed successfully.")
    else:
        print(f"[ROS 2 Action Executor] Action '{action['action_type']}' failed.")
    return success

# --- Main VLA Pipeline Simulation ---

def run_vla_pipeline(voice_command_audio_sim):
    """Simulates the end-to-end VLA pipeline."""
    print("\n--- Starting Full VLA Pipeline Simulation ---")

    # 1. Voice-to-Text (Whisper)
    transcribed_text = simulate_whisper_transcription(voice_command_audio_sim)
    if not transcribed_text or "unrecognized command" in transcribed_text:
        print("[VLA] Error: Voice command not understood. Exiting.")
        return

    # 2. Cognitive Planning (LLM)
    action_plan = simulate_llm_planning(transcribed_text)
    if not action_plan:
        print("[VLA] Error: LLM failed to generate a plan. Exiting.")
        return

    # 3. ROS 2 Action Execution
    print("\n[VLA] Executing plan with ROS 2 actions...")
    for i, action in enumerate(action_plan):
        print(f"\n[VLA] Step {i+1}/{len(action_plan)}:")
        action_success = simulate_ros2_action_execution(action)
        if not action_success and action['action_type'] not in ["log_message"]:
            print(f"[VLA] Action failed: {action['action_type']}. Attempting re-plan (conceptual)...")
            # In a real system, this would trigger re-planning by the LLM
            # For this demo, we just exit on failure for simplicity.
            print("[VLA] Pipeline halted due to action failure.")
            return

    print("\n--- Full VLA Pipeline Simulation Complete ---")
    print("Humanoid robot successfully performed the task (conceptually).")


if __name__ == "__main__":
    print("Welcome to the Mini-Capstone Full VLA Demo!")
    print("This script simulates the Vision-Language-Action pipeline for a humanoid robot.")
    print("Available conceptual voice commands:")
    print("  - 'Go to kitchen and fetch the cup'")
    print("  - 'Find the red block and place it on the table'")
    print("  - 'Move to the charging station'")
    print("  - Any other command will be marked as 'unrecognized'")

    while True:
        user_input = input("\nEnter your conceptual voice command (or 'exit' to quit): \n> ")
        if user_input.lower() == 'exit':
            break
        
        run_vla_pipeline(user_input)

    print("\nDemo finished.")
