![OS](https://img.shields.io/ubuntu/v/ubuntu-wallpapers/noble)
![ROS_2](https://img.shields.io/ros/v/jazzy/rclcpp)
![Gemini](https://img.shields.io/badge/Gemini-AI%20Powered-blue)
![Nav2](https://img.shields.io/badge/Nav2-Social%20Navigation-green)
![Algorithm](https://img.shields.io/badge/Algorithm-KF%20Hybrid%20Tracker-purple)

![Social Navigation Demo in Simulation without using YOLO Pose](Imagesim.png)
![Social Navigation Demo with LIO robot by YOLO Pose + KF + Gemini creating Social wall](Image0.png)
![Social Navigation Demo with LIO robot by YOLO Pose + KF + Gemini finding human position](Image1.png)

---

## System Architecture

> [**Open Interactive Architecture Diagram**](https://abolghasem28.github.io/Social_navigation/architecture.html) — click any node for technical details

![Architecture Diagram](architecture_diagram_demo.png)

---

# Social Navigation Module — Yahboom ROSMASTER & Lio Platform

## Overview

This repository extends the base [Automatic Addison](https://automaticaddison.com) setup for the **ROSMASTER X3** robot by Yahboom. It has been significantly modified to support **social navigation research**, implementing a **KF-based Hybrid Tracker** that combines Generative AI with probabilistic state estimation to solve human tracking issues.

The system separates people position estimation from semantic scene understanding — a design decision motivated by the empirical observation that foundation models exhibit frame-to-frame positional inconsistency unsuitable for spatial tracking. The result is a two-node architecture:

1. **yolo_detector.py** — People detection and 3D position estimation using YOLO11n-pose and Intel RealSense D435
2. **social_navigation.py** — Kalman Filter tracking, Gemini Flash 2.0 social scene understanding, and Nav2 costmap injection via `/social_obstacles`

---

## Live Robot Demonstration

The following video shows the system running on the Lio platform at Idiap Research Institute. The robot correctly routes around two individuals engaged in conversation (social obstacle injected as a red wall in RViz) and passes directly between two non-interacting individuals (green indicator, no obstacle injected).

[▶ Watch Demo Video](Social_navigation_Demo.mp4)

---

## System Architecture Details

The system separates people position estimation from semantic scene understanding:

```
RGB-D Camera (D435)
        │
        ▼
YOLO11n-pose Detector
├── /detected_humans  (PoseArray — 3D positions)
└── /annotated_image  (RGB + bounding boxes + IDs)
        │
        ├──────────────────────────┐
        ▼                          ▼
Kalman Filter Tracker      Gemini Flash 2.0
(3D position estimation)   (Social scene understanding)
        │                          │
        └──────────┬───────────────┘
                   ▼
           Integration Layer
           (ID-consistent mapping)
                   │
                   ▼
        /social_obstacles (PointCloud2)
                   │
                   ▼
            Nav2 Costmap
          (Path planning)
```

---

### People Position Estimation Pipeline

Each detected individual is localised using the shoulder midpoint from YOLO11n-pose keypoints (COCO indices 5 = left shoulder, 6 = right shoulder) and the D435 depth stream. The 2D pixel coordinates $(u, v)$ of the shoulder midpoint are deprojected to 3D camera-frame coordinates using the camera intrinsics:

$$X = \frac{(u - c_x) \cdot Z}{f_x}, \quad Y = \frac{(v - c_y) \cdot Z}{f_y}$$

where $f_x, f_y$ are the focal lengths and $c_x, c_y$ the principal point, obtained from the `/camera/camera/color/camera_info` topic. When shoulder keypoints are not reliably detected (confidence < 0.4), a fallback position at the upper-middle region of the bounding box is used instead.

Detected individuals are sorted left-to-right by bounding box x-coordinate before ID assignment, ensuring consistent correspondence between YOLO IDs visible in the annotated image and Gemini's left-to-right visual ordering.

---

### Kalman Filter Tracking

Each tracked individual maintains a state vector $\mathbf{x} = [x, y, v_x, v_y]^\top$ encoding planar position and velocity in the robot coordinate frame. The filter executes a predict-update cycle at each timestep:

**Prediction:**

$$\hat{\mathbf{x}}_{k|k-1} = \mathbf{A}\hat{\mathbf{x}}_{k-1}, \quad \mathbf{P}_{k|k-1} = \mathbf{A}\mathbf{P}_{k-1}\mathbf{A}^\top + \mathbf{Q}$$

where the state transition matrix is:

$$\mathbf{A} = \begin{bmatrix} 1 & 0 & dt & 0 \\ 0 & 1 & 0 & dt \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}, \quad dt = 0.1 \text{ s}$$

**Update:**

$$\mathbf{y}_k = \mathbf{z}_k - \mathbf{H}\hat{\mathbf{x}}_{k|k-1}$$

$$\mathbf{S}_k = \mathbf{H}\mathbf{P}_{k|k-1}\mathbf{H}^\top + \mathbf{R}$$

$$\mathbf{K}_k = \mathbf{P}_{k|k-1}\mathbf{H}^\top \mathbf{S}_k^{-1}$$

$$\hat{\mathbf{x}}_k = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k\mathbf{y}_k, \quad \mathbf{P}_k = (\mathbf{I} - \mathbf{K}_k\mathbf{H})\mathbf{P}_{k|k-1}$$

where $\mathbf{H} = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \end{bmatrix}$ maps the state to the $(x, y)$ measurement space, $\mathbf{y}_k$ is the innovation, $\mathbf{S}_k$ is the innovation covariance, and $\mathbf{K}_k$ is the Kalman gain.

**Tuned parameters:**

| Parameter | Value | Description |
|-----------|-------|-------------|
| $\mathbf{Q}$ | `diag(0.03, 0.03, 0.1, 0.1)` | Process noise covariance |
| $\mathbf{R}$ | $0.6 \cdot \mathbf{I}_2$ | Measurement noise covariance |
| Friction factor | $0.95$ | Applied to velocity components per step |
| Tracker TTL | $3.0$ s | Timeout before tracker removal |

---

### Semantic Scene Understanding Pipeline

The Gemini Flash 2.0 API is called asynchronously every 2.0 seconds via a `ThreadPoolExecutor` background thread, decoupled from the camera frame rate to prevent API latency from blocking real-time navigation. An API call is made only when:

- At least 2 people are detected in the scene
- 2.0 seconds have elapsed since the previous call

The annotated RGB image (with YOLO bounding boxes and person IDs) is submitted to the Gemini Flash 2.0 API together with a structured natural language prompt. The model returns a JSON object specifying for each unique pair of individuals: an engagement level (low / medium / high), a continuous crossability score $c \in [0.0, 1.0]$, and a short reasoning statement.

---

### Social Obstacle Injection

For each pair of tracked individuals, the navigation decision follows:

$$\text{inject obstacle} = \begin{cases} \text{true} & \text{if } c < 0.5 \\ \text{false} & \text{if } c \geq 0.5 \end{cases}$$

When an obstacle is injected, a wall of `PointCloud2` points is generated spanning the space between the two individuals' estimated positions in the map frame and published to `/social_obstacles`. The Nav2 costmap obstacle layer marks these cells as occupied, causing the global and local planners to route around the interaction space. The `clearing: true` configuration ensures obstacles are removed when the social link is no longer detected.

---

## Setup

### Prerequisites

```bash
# Activate conda environment
conda activate your_env

# Install dependencies
pip install ultralytics google-generativeai opencv-python cv_bridge
```

### Environment Variable

```bash
export GEMINI_API_KEY="your_api_key_here"
```

---

### Running the System

**Terminal 1 — RealSense camera:**
```bash
ros2 launch realsense2_camera rs_launch.py \
    pointcloud.enable:=true \
    align_depth.enable:=true
```

**Terminal 2 — Static TF transform (Lio platform):**
```bash
ros2 run tf2_ros static_transform_publisher \
    -0.10 0 0.052 0 -1.5708 0 \
    lio_gripper_interface_link camera_link
```

**Terminal 3 — YOLO detector:**
```bash
python3 yolo_detector.py
```

**Terminal 4 — Social navigator:**
```bash
python3 social_navigation.py
```

---

## ROS2 Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/camera/camera/color/image_raw` | `Image` | Subscribe | Raw RGB stream from D435 |
| `/camera/camera/aligned_depth_to_color/image_raw` | `Image` | Subscribe | Aligned depth stream |
| `/detected_humans` | `PoseArray` | Publish | 3D human positions in camera frame |
| `/annotated_image` | `Image` | Publish | RGB with bounding boxes and IDs |
| `/social_obstacles` | `PointCloud2` | Publish | Social obstacle walls for Nav2 |
| `/human_markers` | `MarkerArray` | Publish | Cylinder markers for RViz |

---

## Nav2 Configuration

Add `/social_obstacles` as an observation source in your `nav2_params.yaml`. This must be declared at system startup — Nav2 does not support runtime addition of new observation sources.

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan social_obstacles
        scan:
          topic: /scan
          data_type: "LaserScan"
          marking: true
          clearing: true
        social_obstacles:
          topic: /social_obstacles
          data_type: "PointCloud2"
          marking: true
          clearing: true
          min_obstacle_height: 0.0
          max_obstacle_height: 2.0
          obstacle_max_range: 10.0
          obstacle_min_range: 0.0

global_costmap:
  global_costmap:
    ros__parameters:
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan social_obstacles
        social_obstacles:
          topic: /social_obstacles
          data_type: "PointCloud2"
          marking: true
          clearing: true
          min_obstacle_height: 0.0
          max_obstacle_height: 2.0
          obstacle_max_range: 10.0
          obstacle_min_range: 0.0
```

**Important:** The `social_obstacles` section must be inside the `obstacle_layer` block at the same indentation level as `scan`.

---

## Camera Configuration (Yahboom Simulation — Gazebo)

If using the Yahboom ROSMASTER X3 in Gazebo, configure the simulated D435 in the URDF:

```xml
horizontal_fov:=1.5184
image_width:=424
image_height:=240
clip_near:=0.05
clip_far:=12.0
update_rate:=5.0
xyz_offset:='0.105 0 0.05'
rpy_offset:='0 0.2 0'
```

> **Note:** High resolution or update rate can cause simulation instability. Use the values above for stable operation.

---

## Verification Commands

```bash
# Check social obstacles are being published
ros2 topic hz /social_obstacles

# Check subscription count (should be 2 — local + global costmap)
ros2 topic info /social_obstacles

# Verify Nav2 observation sources
ros2 param get /local_costmap/local_costmap obstacle_layer.observation_sources
# Expected: "scan social_obstacles"

# Monitor human detections
ros2 topic echo /detected_humans

# Check Kalman Filter stability (printed every 100 frames in terminal)
# Look for STABILITY REPORT output
```

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Social obstacles not appearing in costmap | Check Nav2 YAML indentation and `clearing: true` |
| Ghost obstacles persisting | Check Kalman Filter TTL (default 3.0 s) and confidence threshold |
| TF lookup warnings | Verify static transform is published before launching social_navigation.py |
| Gemini API errors | Check `GEMINI_API_KEY` environment variable and Wi-Fi connection |
| Robot unstable in simulation | Reduce camera resolution or update rate in URDF |
| ID mismatch between YOLO and Gemini | Ensure left-to-right bounding box sorting is active |
| No detections | Check YOLO confidence threshold (default 0.40) and person class filter |
| Depth values missing | Ensure `align_depth.enable:=true` in RealSense launch |

---

## Repository Structure

```
yahboom_rosmaster_navigation/
├── scripts/
│   ├── yolo_detector.py          # Node 1: YOLO detection + 3D localisation
│   ├──social_navigation.py      # Node 2: KF tracking + Gemini + Nav2 injection
│   └── ...

```

---

## Related Publication

This system is described in detail in:

> Esmaeily, A. (2025). *Zero-Shot Semantic Scene Understanding for Socially Appropriate Robot Navigation using Multimodal Foundation Models*. MSc Thesis, KTH Royal Institute of Technology / Idiap Research Institute.

---

## Author

**Abolghasem Esmaeily**  
MSc Systems, Control and Robotics — KTH Royal Institute of Technology  
Thesis conducted at Idiap Research Institute, Martigny, Switzerland  
Supervisors: Dr. Emmanuel Senft  and Sarah Gillet
