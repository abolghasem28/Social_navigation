#!/bin/bash
# ==============================================================================
# Script: record_scenario.sh
# Purpose: Automates ROS 2 bag recording for semantic social navigation thesis.
# Usage: ./record_scenario.sh <scenario_name>
# Example: ./record_scenario.sh photography
# In order to run a recorded scenario, use the following command:
# ros2 bag play ~/ros2_ws/src/yahboom_rosmaster/thesis_evaluation/rosbags/<saved_bag> -l

# Author: Abolghasem Esmaeily
# ==============================================================================

if [ -z "$1" ]; then
    echo "ERROR: You must provide a scenario name."
    echo "Usage: ./record_scenario.sh <scenario_name>"
    echo "Example: ./record_scenario.sh conversation"
    exit 1
fi

SCENARIO_NAME=$1
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_NAME="${SCENARIO_NAME}_${TIMESTAMP}"

BASE_DIR="$HOME/ros2_ws/src/yahboom_rosmaster/thesis_evaluation/rosbags"

if [ ! -d "$BASE_DIR" ]; then
    echo "Creating directory: $BASE_DIR"
    mkdir -p "$BASE_DIR"
fi

TARGET_PATH="${BASE_DIR}/${BAG_NAME}"

TOPICS=(
    "/camera/camera/color/image_raw"
    "/camera/camera/aligned_depth_to_color/image_raw"
    "/camera/camera/color/camera_info"
    "/tf"
    "/tf_static"
)

echo "====================================================="
echo " STARTING ROSBAG RECORDING"
echo " Scenario: $SCENARIO_NAME"
echo " Saving to: $TARGET_PATH"
echo " Press Ctrl+C to stop recording."
echo "====================================================="

ros2 bag record -o "$TARGET_PATH" "${TOPICS[@]}"

