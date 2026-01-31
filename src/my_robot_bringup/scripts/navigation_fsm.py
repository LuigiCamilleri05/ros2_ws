#!/usr/bin/env python3
"""
Navigation FSM Node - Manages robot navigation states
States: IDLE, NAVIGATING, AVOIDING, REPLANNING, RECOVERING

Subscribes to Nav2 action feedback and publishes current state for monitoring.
Handles recovery behaviors when navigation fails.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy

from enum import Enum
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker


class NavState(Enum):
    """Navigation FSM States"""
    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING"
    AVOIDING = "AVOIDING"
    REPLANNING = "REPLANNING"
    RECOVERING = "RECOVERING"


class NavigationFSM(Node):
    """
    Finite State Machine for robot navigation.
    Monitors Nav2 status and manages recovery behaviors.
    """

    def __init__(self):
        super().__init__('navigation_fsm')
        
        # Parameters
        self.declare_parameter('recovery_timeout', 10.0)
        self.declare_parameter('max_replan_attempts', 3)
        self.declare_parameter('stuck_threshold', 5.0)
        
        self.recovery_timeout = self.get_parameter('recovery_timeout').value
        self.max_replan_attempts = self.get_parameter('max_replan_attempts').value
        self.stuck_threshold = self.get_parameter('stuck_threshold').value
        
        # State management
        self.current_state = NavState.IDLE
        self.previous_state = NavState.IDLE
        self.replan_count = 0
        self.last_position = None
        self.stuck_start_time = None
        self.current_goal = None
        self.goal_handle = None
        
        # Callback group for concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # Publishers
        self.state_pub = self.create_publisher(String, '/navigation_state', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_marker_pub = self.create_publisher(Marker, '/goal_marker', 10)
        self.state_marker_pub = self.create_publisher(Marker, '/nav_state_marker', 10)
        
        # Subscribers
        odom_qos = QoSProfile(depth=10)
        odom_qos.reliability = ReliabilityPolicy.RELIABLE
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, odom_qos,
            callback_group=self.callback_group
        )
        
        # Goal pose subscriber (from RViz "2D Goal Pose")
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10,
            callback_group=self.callback_group
        )
        
        # Nav2 Action Client
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.callback_group
        )
        
        # Timers
        self.state_timer = self.create_timer(0.5, self.publish_state)
        self.stuck_check_timer = self.create_timer(1.0, self.check_if_stuck)
        
        self.get_logger().info('Navigation FSM initialized - State: IDLE')
        self.get_logger().info('Waiting for Nav2 action server...')
        
        # Wait for Nav2 to be ready
        if self.nav_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().info('Nav2 action server ready!')
        else:
            self.get_logger().warn('Nav2 action server not available after 30s')

    def transition_to(self, new_state: NavState):
        """Transition to a new state with logging."""
        if new_state != self.current_state:
            self.previous_state = self.current_state
            self.current_state = new_state
            self.get_logger().info(
                f'State transition: {self.previous_state.value} -> {new_state.value}'
            )

    def publish_state(self):
        """Publish current state for monitoring."""
        msg = String()
        msg.data = self.current_state.value
        self.state_pub.publish(msg)
        
        # Publish state as text marker in RViz
        self.publish_state_marker()
        
        # Publish goal marker if we have a goal
        if self.current_goal:
            self.publish_goal_marker()
    
    def publish_goal_marker(self):
        """Publish a visual marker at the goal position."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'goal'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        # Position at goal
        marker.pose = self.current_goal.pose
        marker.pose.position.z = 0.5  # Raise it above ground
        
        # Size
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 1.0
        
        # Color based on state
        if self.current_state == NavState.IDLE:
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)  # Green = reached
        elif self.current_state == NavState.NAVIGATING:
            marker.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.8)  # Blue = navigating
        elif self.current_state == NavState.REPLANNING:
            marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.8)  # Orange = replanning
        elif self.current_state == NavState.RECOVERING:
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)  # Red = recovering
        else:
            marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)  # Yellow
        
        marker.lifetime.sec = 0  # Persistent
        self.goal_marker_pub.publish(marker)
    
    def publish_state_marker(self):
        """Publish a text marker showing current state."""
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'nav_state'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Position above robot
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.5
        marker.pose.orientation.w = 1.0
        
        # Text
        marker.text = f"State: {self.current_state.value}"
        marker.scale.z = 0.3  # Text height
        
        # Color based on state
        if self.current_state == NavState.IDLE:
            marker.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=1.0)  # Gray
        elif self.current_state == NavState.NAVIGATING:
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Green
        elif self.current_state == NavState.REPLANNING:
            marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)  # Orange
        elif self.current_state == NavState.RECOVERING:
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red
        else:
            marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # Yellow
        
        marker.lifetime.sec = 0
        self.state_marker_pub.publish(marker)

    def odom_callback(self, msg: Odometry):
        """Track robot position for stuck detection."""
        self.last_position = msg.pose.pose.position

    def goal_callback(self, msg: PoseStamped):
        """Handle new goal from RViz or other sources."""
        self.get_logger().info(
            f'Received new goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )
        self.current_goal = msg
        self.replan_count = 0
        self.send_goal(msg)

    def send_goal(self, goal_pose: PoseStamped):
        """Send navigation goal to Nav2."""
        if not self.nav_client.server_is_ready():
            self.get_logger().error('Nav2 action server not ready!')
            return
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        self.get_logger().info('Sending goal to Nav2...')
        self.transition_to(NavState.NAVIGATING)
        
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection."""
        self.goal_handle = future.result()
        
        if not self.goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            self.transition_to(NavState.IDLE)
            return
        
        self.get_logger().info('Goal accepted by Nav2')
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback - detect if replanning needed."""
        feedback = feedback_msg.feedback
        
        # Check distance remaining
        distance = feedback.distance_remaining
        
        # If in AVOIDING state and obstacle cleared, go back to NAVIGATING
        if self.current_state == NavState.AVOIDING:
            self.transition_to(NavState.NAVIGATING)

    def result_callback(self, future):
        """Handle navigation result."""
        result = future.result()
        status = result.status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached successfully!')
            self.transition_to(NavState.IDLE)
            self.replan_count = 0
            
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('Navigation aborted!')
            self.handle_navigation_failure()
            
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Navigation canceled')
            self.transition_to(NavState.IDLE)
            
        else:
            self.get_logger().warn(f'Navigation ended with status: {status}')
            self.transition_to(NavState.IDLE)

    def handle_navigation_failure(self):
        """Handle navigation failure with recovery behaviors."""
        self.replan_count += 1
        self.get_logger().warn(f'Navigation failure #{self.replan_count}')
        
        if self.replan_count <= self.max_replan_attempts:
            # Try replanning
            self.transition_to(NavState.REPLANNING)
            self.get_logger().info('Attempting replan...')
            
            # Wait a moment then retry
            self.create_timer(
                2.0, 
                lambda: self.retry_navigation(),
                callback_group=self.callback_group
            )
        else:
            # Max replans exceeded, try recovery
            self.transition_to(NavState.RECOVERING)
            self.execute_recovery()

    def retry_navigation(self):
        """Retry navigation to current goal."""
        # Check if Nav2 is still available
        if not self.nav_client.server_is_ready():
            self.get_logger().error('Nav2 action server not ready!')
            self.transition_to(NavState.IDLE)
            self.current_goal = None  # Clear goal to stop retries
            return
            
        if self.current_goal and self.current_state == NavState.REPLANNING:
            self.get_logger().info('Retrying navigation...')
            self.send_goal(self.current_goal)

    def execute_recovery(self):
        """Execute recovery behavior (backup and rotate)."""
        self.get_logger().info('Executing recovery: backup and rotate')
        
        # Backup
        twist = Twist()
        twist.linear.x = -0.2
        
        for _ in range(10):  # Backup for ~1 second
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Rotate
        twist.linear.x = 0.0
        twist.angular.z = 0.5
        
        for _ in range(20):  # Rotate for ~2 seconds
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Stop
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        self.get_logger().info('Recovery complete')
        
        # Reset and retry
        self.replan_count = 0
        if self.current_goal:
            self.transition_to(NavState.NAVIGATING)
            self.send_goal(self.current_goal)
        else:
            self.transition_to(NavState.IDLE)

    def check_if_stuck(self):
        """Check if robot is stuck (no movement for threshold time)."""
        if self.current_state != NavState.NAVIGATING:
            self.stuck_start_time = None
            return
        
        if self.last_position is None:
            return
        
        # Simple stuck detection based on position history
        # In a real implementation, you'd track position change over time
        # For now, this is a placeholder that triggers recovery if needed


def main(args=None):
    rclpy.init(args=args)
    
    node = NavigationFSM()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
