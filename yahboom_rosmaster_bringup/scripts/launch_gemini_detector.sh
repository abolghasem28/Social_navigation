#!/bin/bash

# Launch script for Gemini Human Detector
# Usage: ./launch_gemini_detector.sh (No argument needed if key is hardcoded)

# --- HARDCODED KEY ---
export GEMINI_API_KEY=""
# ---------------------


echo "Starting Safety Monitor..."
echo ""

# Source ROS2
#source /opt/ros/jazzy/setup.bash

# The path to Conda Python interpreter
#CONDA_PYTHON="/home/aesmaeily/anaconda3/envs/gemini_env/bin/python"


# Run the node
python3 /home/aesmaeily/ros2_ws/src/yahboom_rosmaster/yahboom_rosmaster_navigation/scripts/social_navigation.py \
#python3 /home/aesmaeily/ros2_ws/src/yahboom_rosmaster/yahboom_rosmaster_navigation/scripts/social_navigation_hybridsim.py \
#python3 /home/aesmaeily/ros2_ws/src/yahboom_rosmaster/yahboom_rosmaster_navigation/scripts/social_navigation_hybridreal.py \
#$CONDA_PYTHON /home/aesmaeily/ros2_ws/src/yahboom_rosmaster/yahboom_rosmaster_navigation/scripts/gemini_analyse.py \

    --ros-args \
    -p gemini_api_key:="$GEMINI_API_KEY" 
   # -p camera_topic:=/cam_1/color/image_raw \
    #-p distance_method:="fusion" \
    # -p human_detection_distance:=1.0 \
    # -p check_frequency:=5.0
    
    #  -p gemini_analysis_delay:=3.0 \
   # -p scan_topic:=/scan \
    #-p fast_check_rate:=10.0 \
    #-p pause_on_any_detection:=true
   