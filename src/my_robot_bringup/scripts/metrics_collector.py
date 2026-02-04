#!/usr/bin/env python3
"""
Navigation Metrics Collector
Logs navigation performance metrics to CSV for analysis.

Metrics tracked:
- Success rate of reaching goal without collision
- Path efficiency (% deviation from optimal path)
- Average time-to-goal
- Number of dynamic replans per trial
- CPU utilization (for efficiency analysis)
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String, Bool
import csv
import os
from datetime import datetime
import math
import psutil
# Metrics file contains all the raw data for analysis and debugging
# Summary file containes aggregate metrics like overall success rate and average time, for quick reference

def calculate_distance(x1: float, y1: float, x2: float, y2: float) -> float:
    """Calculate Euclidean distance between two points."""
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


def calculate_path_length(positions: list) -> float:
    """
    Calculate total path length from a list of (x, y) positions.
    
    Args:
        positions: List of (x, y) tuples representing waypoints
        
    Returns:
        Total path length in meters
    """
    if len(positions) < 2:
        return 0.0
    
    total = 0.0
    for i in range(1, len(positions)):
        total += calculate_distance(
            positions[i-1][0], positions[i-1][1],
            positions[i][0], positions[i][1]
        )
    return total


def calculate_path_efficiency(optimal_distance: float, actual_distance: float) -> float:
    """
    Calculate path efficiency as percentage.
    
    100% = perfect (actual == optimal)
    50% = actual path was twice as long as optimal
    
    Args:
        optimal_distance: Straight-line distance to goal
        actual_distance: Actual distance traveled
        
    Returns:
        Efficiency percentage (0-100+)
    """
    if actual_distance <= 0:
        return 0.0
    return (optimal_distance / actual_distance) * 100


def calculate_success_rate(successful: int, total: int) -> float:
    """
    Calculate success rate as percentage.
    
    Args:
        successful: Number of successful tests
        total: Total number of tests
        
    Returns:
        Success rate percentage (0-100)
    """
    if total <= 0:
        return 0.0
    return (successful / total) * 100


def calculate_average(values: list) -> float:
    """Calculate average of a list of values."""
    if not values:
        return 0.0
    return sum(values) / len(values)


# ============================================================================
# ROS Node
# ============================================================================

class MetricsCollector(Node):
    def __init__(self):
        super().__init__('metrics_collector')
        
        # Metrics storage
        self.current_test = {}
        self.test_results = []
        self.test_count = 0
        
        # Aggregate metrics for success rate and averages
        self.total_tests = 0
        self.successful_tests = 0
        self.total_time_to_goal = 0.0
        self.total_path_efficiency = 0.0
        
        # CPU monitoring
        self.cpu_samples = []
        self.cpu_sample_interval = 0.5  # seconds
        
        # Output file
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_file = os.path.expanduser(f'~/ros2_ws/navigation_metrics_{timestamp}.csv')
        self.summary_file = os.path.expanduser(f'~/ros2_ws/navigation_summary_{timestamp}.csv')
        
        # Initialize CSV with extended metrics
        with open(self.csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'test_id', 'start_time', 'end_time', 'duration_sec',
                'goal_x', 'goal_y', 'start_x', 'start_y',
                'result', 'collision_occurred',
                'optimal_path_length', 'planned_path_length', 'actual_distance_traveled',
                'path_efficiency_pct', 'replan_count', 'recovery_count',
                'avg_cpu_pct', 'max_cpu_pct'
            ])
        
        # State tracking
        self.start_pose = None
        self.current_pose = None
        self.distance_traveled = 0.0
        self.last_pose = None
        self.planned_path_length = 0.0
        self.optimal_path_length = 0.0  # Straight-line distance to goal
        self.replan_count = 0
        self.recovery_count = 0
        self.is_navigating = False
        self.collision_occurred = False
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.path_sub = self.create_subscription(
            Path, '/plan', self.path_callback, 10)
        self.nav_state_sub = self.create_subscription(
            String, '/navigation_state', self.nav_state_callback, 10)
        
        # CPU sampling timer
        self.cpu_timer = self.create_timer(self.cpu_sample_interval, self.sample_cpu)
        
        self.get_logger().info(f'Metrics Collector started!')
        self.get_logger().info(f'Saving to: {self.csv_file}')
        self.get_logger().info('Send goals in RViz to start collecting metrics.')
    
    def odom_callback(self, msg):
        """Track robot position and distance traveled."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.current_pose = (x, y)
        
        if self.is_navigating and self.last_pose:
            self.distance_traveled += calculate_distance(
                self.last_pose[0], self.last_pose[1], x, y
            )
        
        self.last_pose = (x, y)
    
    def sample_cpu(self):
        """Sample CPU usage periodically during navigation."""
        if self.is_navigating:
            cpu_percent = psutil.cpu_percent(interval=None)
            self.cpu_samples.append(cpu_percent)
    
    def goal_callback(self, msg):
        """New goal received - start tracking."""
        self.test_count += 1
        
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        start_x = self.current_pose[0] if self.current_pose else 0
        start_y = self.current_pose[1] if self.current_pose else 0
        
        # Calculate optimal (straight-line) path length
        self.optimal_path_length = calculate_distance(start_x, start_y, goal_x, goal_y)
        
        self.current_test = {
            'test_id': self.test_count,
            'start_time': datetime.now().isoformat(),
            'goal_x': goal_x,
            'goal_y': goal_y,
            'start_x': start_x,
            'start_y': start_y,
        }
        
        self.start_pose = self.current_pose
        self.distance_traveled = 0.0
        self.planned_path_length = 0.0
        self.replan_count = 0
        self.recovery_count = 0
        self.is_navigating = True
        self.collision_occurred = False
        self.cpu_samples = []
        self.nav_start_time = self.get_clock().now()
        
        self.get_logger().info(f'Test #{self.test_count} started: Goal ({goal_x:.2f}, {goal_y:.2f})')
        self.get_logger().info(f'  Optimal path length: {self.optimal_path_length:.2f}m')
    
    def path_callback(self, msg):
        """Track planned path length and replans."""
        if not self.is_navigating:
            return
            
        # Calculate path length using helper function
        positions = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        path_length = calculate_path_length(positions)
        
        if self.planned_path_length > 0:
            self.replan_count += 1
        
        self.planned_path_length = path_length
    
    def nav_state_callback(self, msg):
        """Track navigation state changes."""
        state = msg.data
        
        if state == 'RECOVERING':
            self.recovery_count += 1
            self.get_logger().info(f'Recovery #{self.recovery_count} triggered')
        
        elif state == 'COLLISION':
            self.collision_occurred = True
            self.get_logger().warn('Collision detected!')
            self.finish_test('COLLISION')
        
        elif state == 'IDLE' and self.is_navigating:
            # Navigation finished
            self.finish_test('SUCCESS')
        
        elif state == 'FAILED' and self.is_navigating:
            self.finish_test('FAILED')
        
        elif state == 'REPLANNING' and self.is_navigating:
            self.get_logger().info(f'Replan #{self.replan_count + 1}')
    
    def finish_test(self, result):
        """Save test results to CSV."""
        if not self.current_test:
            return
            
        self.is_navigating = False
        end_time = datetime.now()
        duration = (self.get_clock().now() - self.nav_start_time).nanoseconds / 1e9
        
        # Calculate path efficiency using helper function
        path_efficiency = calculate_path_efficiency(
            self.optimal_path_length, self.distance_traveled
        )
        
        # CPU metrics using helper function
        avg_cpu = calculate_average(self.cpu_samples)
        max_cpu = max(self.cpu_samples) if self.cpu_samples else 0.0
        
        # Update aggregate metrics
        self.total_tests += 1
        if result == 'SUCCESS' and not self.collision_occurred:
            self.successful_tests += 1
            self.total_time_to_goal += duration
            self.total_path_efficiency += path_efficiency
        
        row = [
            self.current_test['test_id'],
            self.current_test['start_time'],
            end_time.isoformat(),
            f'{duration:.2f}',
            f"{self.current_test['goal_x']:.2f}",
            f"{self.current_test['goal_y']:.2f}",
            f"{self.current_test['start_x']:.2f}",
            f"{self.current_test['start_y']:.2f}",
            result,
            self.collision_occurred,
            f'{self.optimal_path_length:.2f}',
            f'{self.planned_path_length:.2f}',
            f'{self.distance_traveled:.2f}',
            f'{path_efficiency:.1f}',
            self.replan_count,
            self.recovery_count,
            f'{avg_cpu:.1f}',
            f'{max_cpu:.1f}'
        ]
        
        with open(self.csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row)
        
        self.get_logger().info(f'Test #{self.current_test["test_id"]} completed: {result}')
        self.get_logger().info(f'  Duration: {duration:.1f}s | Path efficiency: {path_efficiency:.1f}%')
        self.get_logger().info(f'  Optimal: {self.optimal_path_length:.2f}m | Traveled: {self.distance_traveled:.2f}m')
        self.get_logger().info(f'  Replans: {self.replan_count} | Recoveries: {self.recovery_count}')
        self.get_logger().info(f'  CPU: avg={avg_cpu:.1f}% max={max_cpu:.1f}%')
        
        # Log running success rate using helper function
        success_rate = calculate_success_rate(self.successful_tests, self.total_tests)
        self.get_logger().info(f'  Running success rate: {success_rate:.1f}% ({self.successful_tests}/{self.total_tests})')
        
        self.current_test = {}
    
    def save_summary(self):
        """Save aggregate summary metrics."""
        if self.total_tests == 0:
            return
            
        success_rate = calculate_success_rate(self.successful_tests, self.total_tests)
        avg_time = self.total_time_to_goal / self.successful_tests if self.successful_tests > 0 else 0
        avg_efficiency = self.total_path_efficiency / self.successful_tests if self.successful_tests > 0 else 0
        
        with open(self.summary_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['metric', 'value'])
            writer.writerow(['total_tests', self.total_tests])
            writer.writerow(['successful_tests', self.successful_tests])
            writer.writerow(['success_rate_pct', f'{success_rate:.1f}'])
            writer.writerow(['avg_time_to_goal_sec', f'{avg_time:.2f}'])
            writer.writerow(['avg_path_efficiency_pct', f'{avg_efficiency:.1f}'])
        
        self.get_logger().info(f'Summary saved to: {self.summary_file}')


def main(args=None):
    rclpy.init(args=args)
    node = MetricsCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f'Metrics saved to: {node.csv_file}')
        node.save_summary()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
