# examples/ai_robot_nav2_demos/humanoid_nav2_example/process_nav2_output.py

import random
import time

def generate_dummy_nav2_output():
    """
    Generates conceptual Nav2 output data.
    """
    # Simulate a planned path (list of 2D points)
    path = []
    start_x, start_y = random.uniform(-10.0, 10.0), random.uniform(-10.0, 10.0)
    for i in range(random.randint(5, 15)):
        path.append({'x': start_x + i * 0.5, 'y': start_y + i * 0.3})

    # Simulate robot status
    robot_status = random.choice(["IDLE", "PLANNING", "FOLLOWING_PATH", "GOAL_REACHED", "ERROR"])
    
    # Simulate current goal
    goal = {'x': path[-1]['x'], 'y': path[-1]['y'], 'theta': random.uniform(0, 2*3.14159)}

    return {
        'planned_path': path,
        'robot_status': robot_status,
        'current_goal': goal,
        'timestamp': time.time()
    }

def process_nav2_output(nav2_output):
    """
    Processes conceptual Nav2 output data.
    """
    if not nav2_output:
        print("No Nav2 data to process.")
        return

    print("\n--- Processing Nav2 Output ---")
    print(f"Timestamp: {nav2_output['timestamp']:.2f}")
    print(f"Robot Status: {nav2_output['robot_status']}")
    print("Current Goal (x, y, theta):")
    print(f"  x: {nav2_output['current_goal']['x']:.2f}")
    print(f"  y: {nav2_output['current_goal']['y']:.2f}")
    print(f"  theta: {nav2_output['current_goal']['theta']:.2f} rad")
    print(f"Planned Path (first 3 points):")
    for i, point in enumerate(nav2_output['planned_path'][:3]):
        print(f"  {i+1}: x={point['x']:.2f}, y={point['y']:.2f}")
    if len(nav2_output['planned_path']) > 3:
        print(f"  ... ({len(nav2_output['planned_path']) - 3} more points)")
    print("------------------------------\n")

if __name__ == "__main__":
    print("Generating dummy Nav2 output data...")
    for _ in range(2): # Simulate a few updates
        dummy_nav2 = generate_dummy_nav2_output()
        process_nav2_output(dummy_nav2)
        time.sleep(1)
