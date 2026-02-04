# Robotics-2

This project contains a ROS 2 package for running a simulated differential drive robot with autonomous navigation, SLAM, and dynamic obstacle avoidance.

## Prerequisites

- **ROS 2 Jazzy** 
- **Gazebo Harmonic**
- **Nav2** 
- **slam_toolbox**
- **psutil** (for CPU metrics)

### Install Dependencies
```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox
pip install psutil
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
- Nav2 navigation stack (A* global planner + DWB local planner)
- AMCL localization
- RViz visualization
- FSM state manager
- Dynamic obstacles controller
- Metrics collector

### 3. Navigate in RViz
1. **Set Initial Pose**: Click "2D Pose Estimate" → click on map where robot is
2. **Set Goal**: Click "2D Goal Pose" → click destination
3. **Watch**: Robot plans path and navigates autonomously!

---

## Launch Files

| Launch File | Description |
|------------|-------------|
| `navigation.launch.xml` | Full navigation with saved map |
| `slam.launch.xml` | SLAM mapping mode (create new maps) |

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      Navigation Stack                        │
├───────────────┬───────────────┬───────────────┬─────────────┤
│   Perception  │   Mapping     │   Planning    │   Control   │
├───────────────┼───────────────┼───────────────┼─────────────┤
│ LIDAR /scan   │ slam_toolbox  │ Global: A*    │ DWB Local   │
│ RGBD Camera   │ (or AMCL)     │ (NavFn)       │ Planner     │
│ Odometry      │               │               │ (DWA-based) │
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
| `/camera/points_processed` | PointCloud2 | Processed depth camera data |

---

## Configuration Files

| File | Purpose |
|------|---------|
| `config/nav2_params.yaml` | Nav2 parameters (A* planner, DWB controller, costmaps) |
| `config/nav2_rviz.rviz` | RViz visualization config |
| `config/slam_toolbox.yaml` | SLAM Toolbox parameters |
| `config/gazebo_bridge.yaml` | Gazebo-ROS bridge configuration |
| `maps/test_world.yaml` | Saved map for navigation |

---

## Project Structure

```
ros2_ws/
├── src/
│   └── my_robot_bringup/        # Main package
│       ├── launch/
│       │   ├── navigation.launch.xml
│       │   └── slam.launch.xml
│       ├── config/
│       │   ├── nav2_params.yaml
│       │   ├── slam_toolbox.yaml
│       │   └── ...
│       ├── maps/
│       ├── worlds/
│       ├── urdf/                # Robot description (URDF/xacro)
│       │   ├── my_robot.urdf.xacro
│       │   ├── mobile_base.xacro
│       │   ├── camera.xacro
│       │   └── lidar.xacro
│       ├── scripts/
│       │   ├── navigation_fsm.py
│       │   ├── metrics_collector.py
│       │   ├── dynamic_obstacles.py
│       │   ├── odom_to_tf.py
│       │   └── ...
│       └── test/
│           ├── test_metrics_collector.py
│           └── test_navigation_fsm.py
└── README.md
```

---

## Metrics Collection

Navigation metrics are automatically collected when running `navigation.launch.xml`.

### Collected Metrics
| Metric | Description |
|--------|-------------|
| Success Rate | % of goals reached without collision |
| Path Efficiency | % deviation from optimal (straight-line) path |
| Time to Goal | Duration from goal sent to completion |
| Replan Count | Number of dynamic replanning events |
| Recovery Count | Number of recovery behaviors triggered |
| CPU Utilization | Average and max CPU % during navigation |

Results saved to:
- `~/ros2_ws/navigation_metrics_<timestamp>.csv` - Per-test details
- `~/ros2_ws/navigation_summary_<timestamp>.csv` - Aggregate summary

---

## Running Tests

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_bringup
colcon test --packages-select my_robot_bringup
colcon test-result
```

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

