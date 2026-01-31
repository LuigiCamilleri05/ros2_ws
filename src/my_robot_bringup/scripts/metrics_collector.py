#!/usr/bin/env python3
"""
Navigation Metrics Collector
Logs navigation performance metrics to CSV for analysis.
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
import csv
import os
from datetime import datetime
import math


class MetricsCollector(Node):
    def __init__(self):
        super().__init__('metrics_collector')
        
        # Metrics storage
        self.current_test = {}
        self.test_results = []
        self.test_count = 0
        
        # Output file
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_file = os.path.expanduser(f'~/ros2_ws/navigation_metrics_{timestamp}.csv')
        
        # Initialize CSV
        with open(self.csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'test_id', 'start_time', 'end_time', 'duration_sec',
                'goal_x', 'goal_y', 'start_x', 'start_y',
                'result', 'planned_path_length', 'actual_distance_traveled',
                'replan_count', 'recovery_count'
            ])
        
        # State tracking
        self.start_pose = None
        self.current_pose = None
        self.distance_traveled = 0.0
        self.last_pose = None
        self.planned_path_length = 0.0
        self.replan_count = 0
        self.recovery_count = 0
        self.is_navigating = False
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.path_sub = self.create_subscription(
            Path, '/plan', self.path_callback, 10)
        self.nav_state_sub = self.create_subscription(
            String, '/navigation_state', self.nav_state_callback, 10)
        
        self.get_logger().info(f'Metrics Collector started!')
        self.get_logger().info(f'Saving to: {self.csv_file}')
        self.get_logger().info('Send goals in RViz to start collecting metrics.')
    
    def odom_callback(self, msg):
        """Track robot position and distance traveled."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.current_pose = (x, y)
        
        if self.is_navigating and self.last_pose:
            dx = x - self.last_pose[0]
            dy = y - self.last_pose[1]
            self.distance_traveled += math.sqrt(dx*dx + dy*dy)
        
        self.last_pose = (x, y)
    
    def goal_callback(self, msg):
        """New goal received - start tracking."""
        self.test_count += 1
        self.current_test = {
            'test_id': self.test_count,
            'start_time': datetime.now().isoformat(),
            'goal_x': msg.pose.position.x,
            'goal_y': msg.pose.position.y,
            'start_x': self.current_pose[0] if self.current_pose else 0,
            'start_y': self.current_pose[1] if self.current_pose else 0,
        }
        
        self.start_pose = self.current_pose
        self.distance_traveled = 0.0
        self.planned_path_length = 0.0
        self.replan_count = 0
        self.recovery_count = 0
        self.is_navigating = True
        self.nav_start_time = self.get_clock().now()
        
        self.get_logger().info(f'Test #{self.test_count} started: Goal ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
    
    def path_callback(self, msg):
        """Track planned path length and replans."""
        if not self.is_navigating:
            return
            
        # Calculate path length
        path_length = 0.0
        for i in range(1, len(msg.poses)):
            dx = msg.poses[i].pose.position.x - msg.poses[i-1].pose.position.x
            dy = msg.poses[i].pose.position.y - msg.poses[i-1].pose.position.y
            path_length += math.sqrt(dx*dx + dy*dy)
        
        if self.planned_path_length > 0:
            self.replan_count += 1
        
        self.planned_path_length = path_length
    
    def nav_state_callback(self, msg):
        """Track navigation state changes."""
        state = msg.data
        
        if state == 'RECOVERING':
            self.recovery_count += 1
            self.get_logger().info(f'Recovery #{self.recovery_count} triggered')
        
        elif state == 'IDLE' and self.is_navigating:
            # Navigation finished
            self.finish_test('SUCCESS')
        
        elif state == 'REPLANNING' and self.is_navigating:
            self.get_logger().info(f'Replan #{self.replan_count + 1}')
    
    def finish_test(self, result):
        """Save test results to CSV."""
        if not self.current_test:
            return
            
        self.is_navigating = False
        end_time = datetime.now()
        duration = (self.get_clock().now() - self.nav_start_time).nanoseconds / 1e9
        
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
            f'{self.planned_path_length:.2f}',
            f'{self.distance_traveled:.2f}',
            self.replan_count,
            self.recovery_count
        ]
        
        with open(self.csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row)
        
        self.get_logger().info(f'Test #{self.current_test["test_id"]} completed: {result}')
        self.get_logger().info(f'  Duration: {duration:.1f}s | Path: {self.planned_path_length:.2f}m | Traveled: {self.distance_traveled:.2f}m')
        self.get_logger().info(f'  Replans: {self.replan_count} | Recoveries: {self.recovery_count}')
        
        self.current_test = {}


def main(args=None):
    rclpy.init(args=args)
    node = MetricsCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f'Metrics saved to: {node.csv_file}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
