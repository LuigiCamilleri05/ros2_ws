#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    pkg_share = get_package_share_directory('my_robot_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    
    # Paths to files
    urdf_path = os.path.join(get_package_share_directory('my_robot_description'), 
                             'urdf', 'my_robot.urdf.xacro')
    nav2_params_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    slam_params_path = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'nav2_config.rviz')
    world_path = os.path.join(pkg_share, 'worlds', 'test_world.sdf')
    gazebo_bridge_path = os.path.join(pkg_share, 'config', 'gazebo_bridge.yaml')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    
    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='True',
        description='Whether to run SLAM')
    
    # Robot State Publisher
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(urdf_path).read()
        }]
    )
    
    # Gazebo Sim
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [world_path, ' -r']}.items()
    )
    
    # Spawn robot in Gazebo
    spawn_robot_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'my_robot',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.1'],
        output='screen'
    )
    
    # Gazebo Bridge
    bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': gazebo_bridge_path,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    # SLAM Toolbox
    slam_cmd = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_path, {'use_sim_time': use_sim_time}],
        condition=launch.conditions.IfCondition(slam)
    )
    
    # Nav2
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_path
        }.items()
    )
    
    # RViz
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_cmd)
    
    # Add the nodes/launch files to the launch description
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(gazebo_cmd)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(bridge_cmd)
    ld.add_action(slam_cmd)
    ld.add_action(nav2_cmd)
    ld.add_action(rviz_cmd)
    
    return ld