#!/usr/bin/env python3
# Create a simple test map for human-human navigation scenarios
import cv2
import numpy as np

# Your world is simple: ground plane with 2 humans
# Let's create a 10m x 10m map at 0.05m resolution = 200x200 pixels

map_size = 200
map_img = np.ones((map_size, map_size), dtype=np.uint8) * 254  # White = free space

# Optional: Add walls around edges for safety
wall_thickness = 3
map_img[0:wall_thickness, :] = 0      # Top wall
map_img[-wall_thickness:, :] = 0      # Bottom wall  
map_img[:, 0:wall_thickness] = 0      # Left wall
map_img[:, -wall_thickness:] = 0      # Right wall

# Save the map
cv2.imwrite('human_test_map.png', map_img)
print("Map created: human_test_map.png")
print(f"Map size: {map_size}x{map_size} pixels")
print("Resolution: 0.05 m/pixel")
print("World coverage: 10m x 10m")