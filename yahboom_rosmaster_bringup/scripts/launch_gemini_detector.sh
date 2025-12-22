#!/bin/bash

# Launch script for Gemini Human Detector
# Usage: ./launch_gemini_detector.sh (No argument needed if key is hardcoded)

# --- HARDCODED KEY ---
API_KEY="AIzaSyB4MhNyivX5GsBXW8TouTOh6uZRS5MRRww"
# ---------------------

# --- REMOVE THIS INITIAL CHECK BLOCK ---
# if [ -z "$1" ]; then
#     echo "Error: Gemini API key required"
#     echo "Usage: $0 YOUR_GEMINI_API_KEY"
#     echo ""
#     echo "Get your API key from: https://aistudio.google.com/app/apikey"
#     exit 1
# fi
# ---------------------------------------

# The rest of the script remains the same
# echo "Starting Gemini Human Detector..."
# echo "Camera topic: /cam_1/color/image_raw"
# echo "Publishing obstacles to: /virtual_obstacles"
echo "Starting Safety Monitor..."
echo ""

# Source ROS2
#source /opt/ros/jazzy/setup.bash

# Run the node
#python3 /home/aesmaeily/ros2_ws/src/yahboom_rosmaster/yahboom_rosmaster_navigation/scripts/social_navigation_node.py \
python3 /home/aesmaeily/ros2_ws/src/yahboom_rosmaster/yahboom_rosmaster_navigation/scripts/social_navigation.py \
    --ros-args \
    -p gemini_api_key:="$API_KEY" \
    -p camera_topic:=/cam_1/color/image_raw \
    -p distance_method:="fusion" \
    # -p human_detection_distance:=1.0 \
    # -p check_frequency:=5.0
    
    #  -p gemini_analysis_delay:=3.0 \
   # -p scan_topic:=/scan \
    #-p fast_check_rate:=10.0 \
    #-p pause_on_any_detection:=true
   