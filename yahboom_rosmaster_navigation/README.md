# Yahboom ROSMASTER Navigation - Social Navigation Module

## Overview

This navigation package contains two main scripts for virtual obstacle management and AI-powered social navigation:

1. **obstacle_manager.py** - Creates static virtual obstacles for testing
2. **social_navigation.py** - AI-powered human detection and social navigation using Google Gemini

---

## Scripts

### 1. obstacle_manager.py

**Purpose:** Creates virtual obstacles and publishes them as PointCloud2 messages for Nav2's ObstacleLayer. Use this script to test if virtual obstacles appear correctly in RViz and if the robot avoids them.

**Usage:**
```bash
ros2 run yahboom_rosmaster_navigation obstacle_manager.py
```

**Configuration:**
- Obstacles are defined in the code with position (x, y) and radius
- Publishes to `/virtual_obstacles` topic
- Frame: `map`

**Example:**
```python
# Add obstacles in __init__
self.add_obstacle(x=-0.75, y=-7.0, radius=0.8)  # Person A
self.add_obstacle(x=0.75, y=-7.0, radius=0.8)   # Person B
```

---

### 2. gemini_social_navigator.py

**Purpose:** Uses Google Gemini AI foundation model to detect humans in camera images and create appropriate virtual obstacles based on their engagement level.

**Gemini Model Configuration:**
```python
genai.configure(api_key=api_key)
try:
    self.model = genai.GenerativeModel('gemini-2.0-flash')
    self.get_logger().info('✓ Using Gemini 2.0 Flash')
except Exception:
    try:
        self.model = genai.GenerativeModel('gemini-1.5-flash')
        self.get_logger().info('✓ Using Gemini 1.5 Flash')
    except Exception as e:
        self.get_logger().error(f'Failed to initialize Gemini: {e}')
        return
```

**Usage:**
```bash
ros2 run yahboom_rosmaster_navigation gemini_social_navigator.py \
    --ros-args -p gemini_api_key:="YOUR_GEMINI_API_KEY"
```

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `gemini_api_key` | (required) | Your Gemini API key |
| `camera_topic` | `/cam_1/color/image_raw` | Camera image topic |
| `analysis_rate` | 2.0 | Gemini analysis rate (Hz) |
| `obstacle_publish_rate` | 10.0 | Obstacle publishing rate (Hz) |
| `obstacle_ttl` | 5.0 | Obstacle time-to-live (seconds) |
| `detection_distance` | 5.0 | Max detection distance (meters) |

**Engagement Levels → Obstacle Radius:**
| Engagement | Activity | Obstacle Radius |
|------------|----------|-----------------|
| HIGH | Conversation, interacting | 1.2m |
| MEDIUM | Standing, looking around | 0.8m |
| LOW | Walking, passing through | 0.5m |

---

## ⚠️ Important Notes

### Camera Configuration

**If you use this code, you need to modify the camera update rate and resolution carefully!**

High resolution or high update rate can cause:
- Robot movement instability
- Simulation lag
- RViz display issues

**Recommended Camera Settings:**

```xml
<!-- In your robot URDF/xacro file -->
horizontal_fov:=1.2
image_width:=640
image_height:=480
clip_near:=0.1
clip_far:=8.0
update_rate:=5
```

**Camera Settings Reference:**

| Parameter | Low Performance | Balanced | High Quality |
|-----------|-----------------|----------|--------------|
| image_width | 320 | 640 | 1280 |
| image_height | 240 | 480 | 720 |
| update_rate | 2 | 5 | 10 |
| clip_far | 5.0 | 8.0 | 10.0 |

**Camera Orientation (rpy_offset):**

The camera pitch angle is critical for human detection:
```xml
<!-- In rosmaster_x3.urdf.xacro -->
rpy_offset="0 0.1 0"  <!-- 0.1 rad ≈ 6° looking down - RECOMMENDED -->
```

| Pitch Value | Direction |
|-------------|-----------|
| -0.50 | 29° UP (sees ceiling - BAD) |
| 0 | Straight ahead |
| 0.1 | 6° down (RECOMMENDED) |
| 0.2 | 12° down |

### Do NOT Run Both Scripts Simultaneously!

Both scripts publish to `/virtual_obstacles` topic. Running both will cause conflicts:

- ✅ Run **obstacle_manager.py** for testing virtual obstacles
- ✅ Run **social_navigation.py** for AI-powered navigation
- ❌ Do NOT run both at the same time

---

## Nav2 Configuration

Ensure your Nav2 config has `virtual_obstacles` in the obstacle layer:

```yaml
# In nav2_params.yaml
local_costmap:
  local_costmap:
    ros__parameters:
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan virtual_obstacles  # Add virtual_obstacles here!
        scan:
          topic: /scan
          # ... scan params ...
        virtual_obstacles:                    # Add this section
          topic: /virtual_obstacles
          data_type: "PointCloud2"
          marking: true
          clearing: false
          min_obstacle_height: 0.0
          max_obstacle_height: 2.0
          obstacle_max_range: 10.0
          obstacle_min_range: 0.0
```

**Important:** The `virtual_obstacles` section must be INSIDE the `obstacle_layer` block, at the same indentation level as `scan`.

---

## Verification Commands

```bash
# Check if obstacles are being published
ros2 topic hz /virtual_obstacles

# Check subscription count (should be 2 - local + global costmap)
ros2 topic info /virtual_obstacles

# Verify Nav2 configuration
ros2 param get /local_costmap/local_costmap obstacle_layer.observation_sources
# Should return: "scan virtual_obstacles"
```

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Robot unstable/shaking | Reduce camera resolution or update_rate |
| Obstacles not appearing in costmap | Check Nav2 config indentation |
| Subscription count is 1 (not 2) | Fix Nav2 config for both local and global costmap |
| Camera looking at ceiling | Change `rpy_offset` pitch to positive value (e.g., 0.1) |
| TF lookup warnings | Wait for localization to initialize |
| Gemini API errors | Check API key and internet connection |

---

## Author

**Abolghasem Esmaeily**  
Date: December 2025

---

## License

This project is part of the Yahboom ROSMASTER ROS2 package.