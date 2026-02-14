# Yahboom ROSMASTER X3 - Social Navigation Platform

![OS](https://img.shields.io/ubuntu/v/ubuntu-wallpapers/noble)
![ROS_2](https://img.shields.io/ros/v/jazzy/rclcpp)
![Gemini](https://img.shields.io/badge/Gemini-AI%20Powered-blue)
![Nav2](https://img.shields.io/badge/Nav2-Social%20Navigation-green)
![Algorithm](https://img.shields.io/badge/Algorithm-EKF%20Hybrid%20Tracker-purple)

![Social Navigation Demo](Image.png)

---

## Overview

This repository extends the base [Automatic Addison](https://automaticaddison.com) setup for the **ROSMASTER X3** robot by Yahboom. It has been significantly modified to support **social navigation research**, implementing an **EKF-based Hybrid Tracker** that combines Generative AI with probabilistic state estimation to solve human tracking issues.

### Key Modifications

| Component | Changes |
|-----------|---------|
| **Nav2 Configuration** | Tuned for human-aware navigation with virtual obstacle support |
| **Gazebo World Files** | Updated with human models for social interaction scenarios |
| **Foundation Model Integration** | Added Gemini AI modules for human detection and engagement analysis |
| **EKF Hybrid Tracker** | Extended Kalman Filter with Mahalanobis data association for robust tracking |

### Core Scripts

To support both research environments, the logic is split into two specialized scripts:

1. **`social_navigation_hybridsim.py`**: Optimized for **Gazebo Simulation**. It handles TF transforms relative to the simulation `map` frame and synchronizes with simulated camera clocks.

2. **`social_navigation_hybridreal.py`**: Optimized for **Real-World Hardware**. It includes specific handling for the **Intel RealSense D435** drivers, manages real-world sensor noise (mm → m conversion), and handles the `camera_color_optical_frame` transforms directly.

---

## Quick Start

### Prerequisites

```bash
# Install dependencies
sudo apt update
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-realsense2-camera

# Python dependencies for Gemini AI
pip install google-generativeai opencv-python pillow numpy --break-system-packages
```

### Setup Aliases (Recommended)

Add these aliases to your `~/.bashrc` for quick access:

```bash
# Navigation aliases for simulation Gazebo
alias nav1='bash /home/<user>/ros2_ws/src/yahboom_rosmaster/yahboom_rosmaster_bringup/scripts/rosmaster_x3_navigation.sh'

# Social navigation with Gemini AI (EKF Hybrid Tracker)
alias social_nav='bash /home/<user>/ros2_ws/src/yahboom_rosmaster/yahboom_rosmaster_bringup/scripts/launch_gemini_detector.sh'

# Source ROS2 workspace
alias srcros='source /home/<user>/ros2_ws/install/setup.bash'
```

Then reload:

```bash
source ~/.bashrc
```

---

## Running the System

You can run the system in two modes: **Simulation** or **Real-World**.

### Mode 1: Simulation (Gazebo)

Use this mode for testing logic and navigation behavior in a safe virtual environment.

**Terminal 1: Launch Simulation & Nav2**
```bash
nav_sim
# Launches Gazebo, Robot State Publisher, Nav2, and RViz
```

**Terminal 2: Launch Social Tracker (Sim)**
```bash
# Ensure your API Key is set
export GEMINI_API_KEY="AIzaSy...YOUR_KEY"

# Run the Simulation Script
ros2 run yahboom_rosmaster_navigation social_navigation_hybridsim.py
```

### Mode 2: Real-World (Hardware)

Use this mode when connected to the physical robot with the Intel RealSense D435 camera.

**Terminal 1: Launch Camera Drivers**

Start the RealSense camera with pointclouds enabled.
```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true pointcloud.enable:=true
```

**Terminal 2: Launch Real-World Tracker**
```bash
# Ensure your API Key is set
export GEMINI_API_KEY="AIzaSy...YOUR_KEY"

# Run the Real-World Script
ros2 run yahboom_rosmaster_navigation social_navigation_hybridreal.py
```

**Terminal 3: Visualization (Optional)**
```bash
ros2 run rviz2 rviz2
# Set Fixed Frame to: camera_color_optical_frame
# Add MarkerArray topic: /human_markers
# Add PointCloud2 topic: /social_obstacles
```

---

## Social Navigation Features

### Human Detection with Gemini AI

The system uses Google's Gemini foundation model to:

1. **Detect humans** in camera images via bounding boxes
2. **Assess engagement level** (conversation, standing, walking)
3. **Provide measurements** to the EKF tracker for probabilistic state estimation

### Engagement-Based Navigation

| Engagement Level | Human Activity | Obstacle Radius | Robot Behavior |
|------------------|----------------|-----------------|----------------|
| **HIGH** | Conversation, interacting | 0.85m | Wide detour — don't interrupt |
| **MEDIUM** | Standing, looking around | 0.60m | Moderate buffer |
| **LOW** | Walking, passing through | 0.35m | Can pass closer |

### Sample Output

When running the social navigation node, you'll see real-time detection logs:

```
[INFO] [social_navigator_hybrid]: Human 1: Eng=medium Method=DEPTH Sensor Depth=0.81m
[INFO] [social_navigator_hybrid]: Human 2: Eng=high Method=VISUAL FALLBACK Depth=3.01m
[INFO] [social_navigator_hybrid]: Human 1: Eng=medium Method=DEPTH Sensor Depth=0.83m
```

**Log Format:** `Human [ID]: Eng=[ENGAGEMENT] Method=[SOURCE] Depth=[DISTANCE]`

| Field | Description |
|-------|-------------|
| `Human 1/2` | Detected person ID |
| `Eng=high/medium/low` | Engagement level (affects obstacle behavior) |
| `DEPTH Sensor` | Distance from depth camera (accurate) |
| `VISUAL FALLBACK` | Pinhole camera model estimate when depth fails |

---

## Algorithm: EKF Hybrid Tracker

The tracking system evolved through three iterations, each solving specific problems:

| Version | Method | Problem Solved | Remaining Issue |
|---------|--------|----------------|-----------------|
| V1 | Velocity Clamping + Low-Pass Filter | Reduced teleportation jumps | Jitter and ghosting persisted |
| V2 | V1 + Hit Threshold | Eliminated ghost obstacles | 2s delay before new detections appear |
| **V3 (Current)** | **Extended Kalman Filter** | **Probabilistic smoothing + motion prediction** | **None — best stability and accuracy** |

### Why EKF?

The fundamental problem is that Gemini's per-frame analysis produces **non-deterministic measurements**: the same person in the same position yields slightly different bounding boxes each frame due to pixel-level variations in color and illumination. Combined with noisy depth sensor data, this causes obstacle positions to jitter and ghost.

The EKF addresses both issues:
1. **Process noise** models Gemini's frame-to-frame variation
2. **Measurement noise** models the depth sensor uncertainty
3. **State prediction** enables smooth obstacle movement between detection frames

A Particle Filter was considered but rejected due to computational cost — Gemini already introduces latency (~1s per frame), and particle filtering on a laptop CPU would compound this significantly.

---

### 1. EKF State Model

Each tracked human is modeled with a **constant-velocity state vector**:

$$\mathbf{x} = \begin{bmatrix} x \\ y \\ v_x \\ v_y \end{bmatrix}$$

where $(x, y)$ is the position in the map frame and $(v_x, v_y)$ is the estimated velocity.

#### Prediction Step

The state transition follows a constant-velocity model:

$$\mathbf{x}_{k|k-1} = \mathbf{F} \cdot \mathbf{x}_{k-1|k-1}$$

$$\mathbf{F} = \begin{bmatrix} 1 & 0 & \Delta t & 0 \\ 0 & 1 & 0 & \Delta t \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

The predicted covariance:

$$\mathbf{P}_{k|k-1} = \mathbf{F} \cdot \mathbf{P}_{k-1|k-1} \cdot \mathbf{F}^T + \mathbf{Q}$$

where:

$$\mathbf{Q} = \begin{bmatrix} 0.01 & 0 & 0 & 0 \\ 0 & 0.01 & 0 & 0 \\ 0 & 0 & 0.05 & 0 \\ 0 & 0 & 0 & 0.05 \end{bmatrix}$$

The higher process noise on velocity $(0.05)$ versus position $(0.01)$ reflects that velocity changes are expected between frames (people start and stop), while position should evolve smoothly.

A **velocity decay factor** of $0.85$ is applied after each prediction to bias stationary humans toward zero velocity:

$$v_x \leftarrow 0.85 \cdot v_x, \quad v_y \leftarrow 0.85 \cdot v_y$$

#### Update Step

When a new Gemini detection arrives, the measurement is the position in the map frame:

$$\mathbf{z}_k = \begin{bmatrix} x_{measured} \\ y_{measured} \end{bmatrix}$$

The measurement matrix extracts position from the state:

$$\mathbf{H} = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \end{bmatrix}$$

**Innovation** (difference between measurement and prediction):

$$\mathbf{y}_k = \mathbf{z}_k - \mathbf{H} \cdot \mathbf{x}_{k|k-1}$$

**Innovation covariance**:

$$\mathbf{S}_k = \mathbf{H} \cdot \mathbf{P}_{k|k-1} \cdot \mathbf{H}^T + \mathbf{R}$$

where $\mathbf{R} = \mathbf{I}_2 \cdot 1.0$ represents the measurement noise. The value $R = 1.0$ indicates high measurement uncertainty, meaning the filter trusts its own prediction more than any single Gemini detection.

**Kalman Gain**:

$$\mathbf{K}_k = \mathbf{P}_{k|k-1} \cdot \mathbf{H}^T \cdot \mathbf{S}_k^{-1}$$

**State update**:

$$\mathbf{x}_{k|k} = \mathbf{x}_{k|k-1} + \mathbf{K}_k \cdot \mathbf{y}_k$$

**Covariance update**:

$$\mathbf{P}_{k|k} = (\mathbf{I}_4 - \mathbf{K}_k \cdot \mathbf{H}) \cdot \mathbf{P}_{k|k-1}$$

#### Z-Axis (Depth) Smoothing

The depth coordinate is filtered separately with a low-pass filter since vertical motion is not modeled by the 2D EKF:

$$z_t = 0.85 \cdot z_{t-1} + 0.15 \cdot z_{measured}$$

---

### 2. Mahalanobis Distance for Data Association

A critical component is **matching new detections to existing tracks**. Euclidean distance treats all directions equally, which fails when tracks have different uncertainty shapes. Instead, we use **Mahalanobis distance**, which weights the residual by the inverse of the innovation covariance.

#### Why Not Euclidean Distance?

| Property | Euclidean | Mahalanobis |
|----------|-----------|-------------|
| Uses covariance? | No — fixed radius gate | Yes — adaptive elliptical gate |
| Handles anisotropic uncertainty? | No | Yes |
| Adapts to track confidence? | No | Yes — new tracks have wider gates |
| Ghost track prevention | Poor | Strong — rejects unlikely matches |

#### The Formula

For a detection $\mathbf{z}$ and a track with predicted state $\mathbf{x}$:

$$d^2_M = \mathbf{y}^T \cdot \mathbf{S}^{-1} \cdot \mathbf{y}$$

where:
- $\mathbf{y} = \mathbf{z} - \mathbf{H} \cdot \mathbf{x}$ is the innovation residual
- $\mathbf{S} = \mathbf{H} \cdot \mathbf{P} \cdot \mathbf{H}^T + \mathbf{R}$ is the innovation covariance

#### Chi-Squared Gating

Under the assumption that the innovation is Gaussian, $d^2_M$ follows a **chi-squared distribution** with degrees of freedom equal to the measurement dimension (2 for our x, y measurements).

We apply a **99% confidence gate**:

$$d^2_M < \chi^2_{2, 0.99} = 9.21$$

This means we only accept an association if it falls within the 99th percentile of the expected distribution. Detections outside this gate are treated as **new tracks** rather than being incorrectly assigned to existing ones.

| Confidence Level | $\chi^2$ Threshold (2 DOF) | Meaning |
|------------------|---------------------------|---------|
| 95% | 5.99 | Tighter gate — may miss valid associations |
| **99%** | **9.21** | **Used in our system — good balance** |
| 99.5% | 10.60 | Looser gate — may allow false associations |

#### Implementation

```python
# Innovation residual
y_res = z_meas - H @ t.state

# Innovation covariance
S = H @ t.P @ H.T + t.R

# Mahalanobis distance squared
d2 = y_res.T @ np.linalg.inv(S) @ y_res
d2_val = float(d2[0, 0])

# 99% chi-squared gate (2 DOF)
if d2_val < 9.21 and d2_val < min_dist:
    best_tracker = t
```

---

### 3. Legacy Methods (V1 — Retained as Fallback)

The original Velocity Clamping and Exponential Smoothing methods are retained in the simulation script as a simpler alternative.

#### Velocity Clamping (Anti-Teleportation)

Limits the maximum displacement per time step to prevent impossible jumps:

$$\mathbf{P}_{clamped} = \begin{cases} \mathbf{P}_{new} & \text{if } d \le d_{max} \\ \mathbf{P}_{old} + \frac{\Delta \mathbf{P}}{|\Delta \mathbf{P}|} \cdot d_{max} & \text{if } d > d_{max} \end{cases}$$

where $d = |\mathbf{P}_{new} - \mathbf{P}_{old}|$ and $d_{max} = 0.3\text{m}$.

#### Exponential Smoothing (Low-Pass Filter)

Blends the current measurement with the previous state:

$$\mathbf{P}_{t} = \alpha \cdot \mathbf{P}_{input} + (1 - \alpha) \cdot \mathbf{P}_{t-1}$$

where $\alpha = 0.2$.

#### Combined Legacy Formula

$$\mathbf{P}_{final} = \alpha \cdot \left[ \mathbf{P}_{old} + \min\left(1, \frac{d_{max}}{|\mathbf{P}_{sensor} - \mathbf{P}_{old}|} \right) (\mathbf{P}_{sensor} - \mathbf{P}_{old}) \right] + (1 - \alpha) \cdot \mathbf{P}_{old}$$

---

## Configuration

### Camera Settings

Camera settings affect both detection quality and system performance. Edit in `rosmaster_x3.urdf.xacro`:

```xml
<xacro:rgbd_camera
  xyz_offset="0.105 0 0.05"
  rpy_offset="0 0.1 0"/>
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
  observation_sources: scan social_obstacles
  scan:
    topic: /scan
    # ... scan params ...
  social_obstacles:
    topic: /social_obstacles
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
export GZ_SIM_RESOURCE_PATH=/home/<user>/ros2_ws/src/yahboom_rosmaster/yahboom_rosmaster_gazebo/models:$GZ_SIM_RESOURCE_PATH

# ROS2 workspace
source /home/<user>/ros2_ws/install/setup.bash
```

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| **Robot sees ghosts (Double Obstacles)** | Ensure you are running the latest EKF-based script, not the legacy version |
| **Social obstacles not appearing** | Check Nav2 config — `social_obstacles` must be inside `obstacle_layer` |
| **Depth always shows VISUAL FALLBACK** | RealSense returns mm, not meters — ensure depth filter uses `> 100` and `< 8000` with `/1000.0` conversion |
| **Transform errors** | Verify TF chain: `map → base_link → camera_link → camera_color_optical_frame`. Run `ros2 run tf2_tools view_frames` |
| **Camera looking at ceiling** | Change `rpy_offset` to `"0 0.1 0"` in URDF |
| **System slow / frozen image** | Throttle debug image publishing — avoid `imgmsg_to_cv2` at 10Hz on the robot |
| **Gemini API errors** | Verify `export GEMINI_API_KEY` is set in the launch script |
| **RealSense not detected** | Check USB connection, run `rs-enumerate-devices` |

---

## Documentation

| Document | Description |
|----------|-------------|
| [Navigation README](yahboom_rosmaster_navigation/README.md) | Virtual obstacles and social navigation guide |
| [Troubleshooting Report](yahboom_rosmaster_navigation/TROUBLESHOOTING_REPORT.md) | Common issues and solutions |
| [Supervisor Report](yahboom_rosmaster_navigation/SUPERVISOR_REPORT.md) | Technical details for academic review |

---

## Credits

- **Base Setup:** [Automatic Addison](https://automaticaddison.com) — ROSMASTER X3 ROS 2 tutorials
- **Robot Hardware:** [Yahboom](https://www.yahboom.net/) — ROSMASTER X3 robot platform
- **AI Integration:** Google Gemini API for human detection and engagement analysis
- **Algorithm Design:** Abolghasem Esmaeily

---

## Author

**Abolghasem Esmaeily**  
Social Navigation Research — MSc Thesis  
KTH Royal Institute of Technology / Idiap Research Institute  
February 2026