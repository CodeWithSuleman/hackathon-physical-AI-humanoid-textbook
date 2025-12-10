# examples/digital_twin_sensors/depth_camera_sim_example/process_depth_data.py

import random
import numpy as np

def generate_dummy_depth_image(width=64, height=48, min_depth=0.5, max_depth=10.0):
    """
    Generates a conceptual depth image with dummy data.
    """
    # Simulate a depth image as a 2D numpy array
    depth_image = np.random.uniform(min_depth, max_depth, (height, width))
    
    # Simulate a closer object in the center
    center_y, center_x = height // 2, width // 2
    for y in range(center_y - 5, center_y + 5):
        for x in range(center_x - 5, center_x + 5):
            if 0 <= y < height and 0 <= x < width:
                depth_image[y, x] = random.uniform(min_depth, 2.0)
                
    return depth_image

def process_depth_image(depth_image):
    """
    Processes a depth image, calculating average depth and identifying the closest point.
    """
    if depth_image is None or depth_image.size == 0:
        print("No depth image data to process.")
        return

    min_depth = np.min(depth_image)
    avg_depth = np.mean(depth_image)
    
    # Find coordinates of the closest point
    min_coords = np.unravel_index(np.argmin(depth_image), depth_image.shape)

    print("\n--- Processing Depth Image ---")
    print(f"Image dimensions: {depth_image.shape[1]}x{depth_image.shape[0]}")
    print(f"Average depth: {avg_depth:.2f}m")
    print(f"Closest point: {min_depth:.2f}m at pixel coordinates {min_coords}")
    
    # You could imagine more complex processing here, like segmentation or obstacle detection.
    
    print("-----------------------------\
")

if __name__ == "__main__":
    print("Generating dummy depth image data...")
    dummy_depth = generate_dummy_depth_image()
    process_depth_image(dummy_depth)
