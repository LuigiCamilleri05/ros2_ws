# Robotics-2

This project contains ROS 2 packages for running a simulated robot with camera-based following behavior and dynamic obstacles.

## Prerequisites

- **ROS 2 Jazzy** 
- **Gazebo Harmonic**
- **Nav2** 
- **slam_toolbox** 

### Install Dependencies
```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox
```

---

## Quick Start

### 0. Kill Any Running Processes (if needed)
```bash
pkill -9 gz; pkill -9 ruby; pkill -9 rviz2
```

### 1. Build the Workspace
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 2. Launch Navigation (with saved map)
```bash
ros2 launch my_robot_bringup navigation.launch.xml
```

This launches:
- Gazebo simulation with the robot
- Nav2 navigation stack
- AMCL localization
- RViz visualization
- FSM state manager

### 3. Start Dynamic Obstacles (separate terminal)
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run my_robot_control dynamic_obstacles
```

### 4. Navigate in RViz
1. **Set Initial Pose**: Click "2D Pose Estimate" → click on map where robot is
2. **Set Goal**: Click "2D Goal Pose" → click destination
3. **Watch**: Robot plans path and navigates autonomously!

---

## Launch Files

| Launch File | Description |
|------------|-------------|
| `navigation.launch.xml` | Full navigation with saved map |
| `slam.launch.xml` | SLAM mapping mode (create new maps) |
| `gazebo.launch.xml` | Gazebo only (no navigation) |

---

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      Navigation Stack                        │
├───────────────┬───────────────┬───────────────┬─────────────┤
│   Sensing     │   Mapping     │   Planning    │   Control   │
├───────────────┼───────────────┼───────────────┼─────────────┤
│ LIDAR /scan   │ slam_toolbox  │ Nav2 Planner  │ DWB Local   │
│ Camera        │ (or AMCL)     │ (NavFn A*)    │ Planner     │
│ Odometry      │               │               │             │
├───────────────┴───────────────┴───────────────┴─────────────┤
│                     FSM State Manager                        │
│           IDLE → NAVIGATING → AVOIDING → RECOVERING          │
└─────────────────────────────────────────────────────────────┘
```

---

## Key Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | Twist | Velocity commands |
| `/scan` | LaserScan | LIDAR data |
| `/odom` | Odometry | Robot odometry |
| `/map` | OccupancyGrid | Map from AMCL |
| `/navigation_state` | String | FSM state (IDLE, NAVIGATING, etc.) |
| `/goal_marker` | Marker | Visual goal indicator in RViz |

---

## Configuration Files

| File | Purpose |
|------|---------|
| `config/nav2_params.yaml` | Nav2 parameters (planners, costmaps) |
| `config/nav2_rviz.rviz` | RViz visualization config |
| `config/slam_params.yaml` | SLAM Toolbox parameters |
| `maps/test_world.yaml` | Saved map for navigation |

---

## Troubleshooting

### Kill stuck processes
```bash
pkill -9 gz; pkill -9 ruby; pkill -9 rviz2
```

### Robot doesn't move
1. Check if AMCL is localized (LIDAR should match map walls in RViz)
2. Set a new "2D Pose Estimate" to help localization

### "Failed to create plan" error
- Goal may be in an obstacle or unknown area
- Try a goal closer to mapped regions

### VirtualBox performance issues
- Simulation may be slow; this is normal
- Timing warnings are cosmetic and don't affect navigation

---

## Project Structure

```
ros2_ws/
├── src/
│   ├── my_robot_description/    # Robot URDF, meshes
│   ├── my_robot_bringup/        # Launch files, configs
│   │   ├── launch/
│   │   ├── config/
│   │   ├── maps/
│   │   ├── worlds/
│   │   └── scripts/
│   └── my_robot_control/        # Control nodes
└── README.md
```

---

## Metrics Collection

Collect navigation performance metrics for analysis:

### Run Metrics Collector
```bash
# In a separate terminal while navigation is running:
source ~/ros2_ws/install/setup.bash
ros2 run my_robot_bringup metrics_collector.py
```

### Collected Metrics
| Metric | Description |
|--------|-------------|
| Duration | Time from goal sent to completion |
| Planned Path Length | Length of planned path (meters) |
| Actual Distance | Distance robot actually traveled |
| Replan Count | Number of path replanning events |
| Recovery Count | Number of recovery behaviors triggered |

Results saved to: `~/ros2_ws/navigation_metrics_YYYYMMDD_HHMMSS.csv`

---

