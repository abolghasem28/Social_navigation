# Yahboom ROSMASTER X3 - Social Navigation Platform

![OS](https://img.shields.io/ubuntu/v/ubuntu-wallpapers/noble)
![ROS_2](https://img.shields.io/ros/v/jazzy/rclcpp)
![Gemini](https://img.shields.io/badge/Gemini-AI%20Powered-blue)
![Nav2](https://img.shields.io/badge/Nav2-Social%20Navigation-green)
![alt text](image.png)

---

## Overview

This repository extends the base [Automatic Addison](https://automaticaddison.com) setup for the **ROSMASTER X3** robot by Yahboom. It has been significantly modified to support **social navigation research**, combining the standard ROS 2 navigation stack with modern AI approaches.

### Key Modifications

| Component | Changes |
|-----------|---------|
| **Nav2 Configuration** | Tuned for human-aware navigation with virtual obstacle support |
| **Gazebo World Files** | Updated with human models for social interaction scenarios |
| **Foundation Model Integration** | Added Gemini AI modules for human detection and engagement analysis |
| **Virtual Obstacles** | Custom PointCloud2-based obstacle system for dynamic human avoidance |

---

## Demo

| Gazebo Simulation | RViz Visualization |
|-------------------|-------------------|
| ![ROSMASTER X3 in Gazebo](https://automaticaddison.com/wp-content/uploads/2024/11/gazebo-800-square-mecanum-controller.gif) | ![ROSMASTER X3 in RViz](https://automaticaddison.com/wp-content/uploads/2024/11/rviz-800-square-mecanum-controller.gif) |

---

## Quick Start

### Prerequisites

```bash
# Install dependencies
sudo apt update
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup

# Python dependencies for Gemini AI
pip install google-generativeai opencv-python pillow --break-system-packages
```

### Setup Aliases (Recommended)

Add these aliases to your `~/.bashrc` for quick access:

```bash
# Navigation aliases
alias nav1='bash /home/aesmaeily/ros2_ws/src/yahboom_rosmaster/yahboom_rosmaster_bringup/scripts/rosmaster_x3_navigation.sh'

# Social navigation with Gemini AI
alias social_nav='bash /home/aesmaeily/ros2_ws/src/yahboom_rosmaster/yahboom_rosmaster_bringup/scripts/launch_gemini_detector.sh'

# Source ROS2 workspace
alias srcros='source /home/aesmaeily/ros2_ws/install/setup.bash'
```

Then reload:
```bash
source ~/.bashrc
```

---

## Running the System

### Option 1: Standard Navigation (Without AI)

```bash
# Terminal 1: Launch complete navigation stack
nav1

# This launches:
# - Gazebo simulation with cafe world
# - Robot state publisher
# - Nav2 navigation stack
# - RViz visualization
```

### Option 2: Social Navigation with Gemini AI

#### Step 1: Configure Gemini API Key

Edit the launch script to add your API key:

```bash
nano /home/aesmaeily/ros2_ws/src/yahboom_rosmaster/yahboom_rosmaster_bringup/scripts/launch_gemini_detector.sh
```

Find and replace `YOUR_API_KEY_HERE` with your actual Gemini API key:

```bash
# Get your API key from: https://aistudio.google.com/app/apikey
API_KEY="YOUR_ACTUAL_GEMINI_API_KEY"
```

#### Step 2: Launch the System

```bash
# Terminal 1: Launch navigation stack
nav1

# Terminal 2: Launch Gemini social navigation (after nav1 is fully loaded)
social_nav
```

#### Alternative: Run Social Navigation Manually

```bash
# Launch social navigation with API key as parameter
ros2 run yahboom_rosmaster_navigation social_navigation.py \
    --ros-args -p gemini_api_key:="YOUR_API_KEY"
```

---

## Package Structure

```
yahboom_rosmaster/
├── yahboom_rosmaster_bringup/
│   └── scripts/
│       ├── rosmaster_x3_navigation.sh    # Main navigation launch script
│       └── launch_gemini_detector.sh     # Gemini AI social navigation script
│
├── yahboom_rosmaster_navigation/
│   ├── scripts/
│   │   ├── obstacle_manager.py           # Static virtual obstacles (testing)
│   │   └── social_navigation.py          # Gemini AI social navigation
│   ├── config/
│   │   └── rosmaster_x3_nav2_default_params.yaml  # Nav2 configuration
│   └── README.md                         # Navigation documentation
│
├── yahboom_rosmaster_gazebo/
│   ├── worlds/
│   │   └── cafe.world                    # Cafe environment with humans
│   └── models/                           # Gazebo models (cafe, tables, etc.)
│
└── yahboom_rosmaster_description/
    └── urdf/
        ├── robots/rosmaster_x3.urdf.xacro    # Robot description
        └── sensors/rgbd_camera.urdf.xacro    # Camera configuration
```

---

## Social Navigation Features

### Human Detection with Gemini AI

The system uses Google's Gemini foundation model to:

1. **Detect humans** in camera images
2. **Assess engagement level** (conversation, standing, walking)
3. **Create virtual obstacles** with appropriate buffer sizes

### Engagement-Based Navigation

| Engagement Level | Human Activity | Obstacle Radius | Robot Behavior |
|------------------|----------------|-----------------|----------------|
| **HIGH** | Conversation, interacting | 1.2m | Wide detour - don't interrupt |
| **MEDIUM** | Standing, looking around | 0.8m | Moderate buffer |
| **LOW** | Walking, passing through | 0.5m | Can pass closer |

### Virtual Obstacles

The system publishes `PointCloud2` messages to `/virtual_obstacles` topic, which Nav2's costmap recognizes as obstacles.

```bash
# Verify virtual obstacles are working
ros2 topic info /virtual_obstacles
# Should show: Subscription count: 2
```

---

## Configuration

### Camera Settings

Camera settings affect both detection quality and system performance. Edit in `rosmaster_x3.urdf.xacro`:

```xml
<xacro:rgbd_camera
  xyz_offset="0.105 0 0.05"
  rpy_offset="0 0.1 0"/>    <!-- 0.1 rad ≈ 6° down - optimal for human detection -->
```

| Parameter | Recommended | Description |
|-----------|-------------|-------------|
| `image_width` | 640 | Image width in pixels |
| `image_height` | 480 | Image height in pixels |
| `update_rate` | 5 | Frames per second |
| `clip_far` | 8.0 | Maximum detection range (meters) |

### Nav2 Virtual Obstacles Configuration

Ensure your `nav2_params.yaml` has virtual obstacles configured:

```yaml
obstacle_layer:
  plugin: "nav2_costmap_2d::ObstacleLayer"
  observation_sources: scan virtual_obstacles
  scan:
    topic: /scan
    # ... scan params ...
  virtual_obstacles:              # Must be INSIDE obstacle_layer!
    topic: /virtual_obstacles
    data_type: "PointCloud2"
    marking: true
    clearing: false
    min_obstacle_height: 0.0
    max_obstacle_height: 2.0
```

---

## Environment Variables

Add to `~/.bashrc`:

```bash
# Gazebo model path (required for cafe world)
export GZ_SIM_RESOURCE_PATH=/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/yahboom_rosmaster_gazebo/models:$GZ_SIM_RESOURCE_PATH

# ROS2 workspace
source /home/aesmaeily/ros2_ws/install/setup.bash
```

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Virtual obstacles not appearing | Check Nav2 config indentation - `virtual_obstacles` must be inside `obstacle_layer` |
| Camera looking at ceiling | Change `rpy_offset` to `"0 0.1 0"` in URDF |
| Robot unstable in RViz | Reduce camera `update_rate` to 5 Hz |
| Gazebo model errors | Set `GZ_SIM_RESOURCE_PATH` environment variable |
| Gemini API errors | Verify API key in launch script |

For detailed troubleshooting, see: `yahboom_rosmaster_navigation/TROUBLESHOOTING_REPORT.md`

---

## Documentation

| Document | Description |
|----------|-------------|
| [Navigation README](yahboom_rosmaster_navigation/README.md) | Virtual obstacles and social navigation guide |
| [Troubleshooting Report](yahboom_rosmaster_navigation/TROUBLESHOOTING_REPORT.md) | Common issues and solutions |
| [Supervisor Report](yahboom_rosmaster_navigation/SUPERVISOR_REPORT.md) | Technical details for academic review |

---

## API Key Setup

To use Gemini AI social navigation:

1. Get your API key from [Google AI Studio](https://aistudio.google.com/app/apikey)
2. Edit the launch script:
   ```bash
   nano /home/aesmaeily/ros2_ws/src/yahboom_rosmaster/yahboom_rosmaster_bringup/scripts/launch_gemini_detector.sh
   ```
3. Replace `YOUR_API_KEY_HERE` with your actual key
4. Save and run `social_nav`

---

## Credits

- **Base Setup:** [Automatic Addison](https://automaticaddison.com) - ROSMASTER X3 ROS 2 tutorials
- **Robot Hardware:** [Yahboom](https://www.yahboom.net/) - ROSMASTER X3 robot platform
- **AI Integration:** Google Gemini API for human detection and engagement analysis
- **Social Navigation Extensions:** Abolghasem Esmaeily

---

## License

This project builds upon the Yahboom ROSMASTER ROS 2 packages. See individual package licenses for details.

---

## Author

**Abolghasem Esmaeily**  
Social Navigation Research  
December 2025