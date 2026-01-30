# Robotics-2

This project contains ROS 2 packages for running a simulated robot with camera-based following behavior and dynamic obstacles.

## Prerequisites
- ROS 2 installed and sourced
- Gazebo installed
- Workspace built with `colcon build`

## Running the Latest `camerafollower.zip`

Follow the steps below in separate terminals (with your ROS 2 environment sourced):

### 1. Launch the Gazebo Simulation
```bash
ros2 launch my_robot_bringup gazebo.launch.xml
```

### 2. Start the Camera Follower Node
```bash
ros2 run my_robot_control camera_follower
```

### 3. Start the Dynamic Obstacles Node
```bash
ros2 run my_robot_control dynamic_obstacles
```

## Notes

Ensure all pakcages are built before running the simulation.
```bash
colcon build
source install/setup.bash
```


