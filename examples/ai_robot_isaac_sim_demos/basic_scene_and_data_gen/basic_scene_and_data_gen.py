# examples/ai_robot_isaac_sim_demos/basic_scene_and_data_gen/basic_scene_and_data_gen.py

import time
import random

def load_isaac_sim_scene(scene_name):
    """
    Conceptual function to simulate loading a scene in Isaac Sim.
    """
    print(f"Simulating Isaac Sim scene load: '{scene_name}.usd'...")
    time.sleep(1) # Simulate loading time
    print(f"Scene '{scene_name}.usd' loaded successfully.")

def generate_synthetic_data(data_type, num_frames=1):
    """
    Conceptual function to simulate synthetic data generation in Isaac Sim.
    """
    print(f"Simulating generating {data_type} synthetic data for {num_frames} frame(s)...")
    for frame in range(num_frames):
        # Dummy data generation for illustration
        if data_type == "RGB":
            dummy_data = f"RGB_Frame_{frame+1}_{random.randint(0, 255)}_{random.randint(0, 255)}_{random.randint(0, 255)}"
        elif data_type == "Depth":
            dummy_data = f"Depth_Frame_{frame+1}_{random.uniform(0.1, 10.0):.2f}m"
        elif data_type == "Semantic Segmentation":
            dummy_data = f"Semantic_Frame_{frame+1}_Object_{random.randint(1,5)}_Class_{random.randint(1,10)}"
        else:
            dummy_data = f"Generic_Synthetic_Data_Frame_{frame+1}"
        
        print(f"  Generated {data_type} data for frame {frame+1}: {dummy_data}")
        time.sleep(0.1)
    print(f"Synthetic {data_type} data generation complete.")

if __name__ == "__main__":
    print("--- Isaac Sim Basic Scene and Synthetic Data Generation Demo ---")

    # Simulate loading a scene
    scene_name = "factory_warehouse_01"
    load_isaac_sim_scene(scene_name)

    # Simulate generating different types of synthetic data
    print("\nStarting synthetic data generation examples:")
    generate_synthetic_data("RGB", num_frames=2)
    generate_synthetic_data("Depth", num_frames=2)
    generate_synthetic_data("Semantic Segmentation", num_frames=1)

    print("\n--- Demo Complete ---")
