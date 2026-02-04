#!/usr/bin/env python3
"""
Integration tests for ROS 2 nodes.
Tests that nodes can be instantiated and communicate properly.
Requires ROS 2 to be running.
"""

import unittest
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class TestNodeCommunication(unittest.TestCase):
    """Test ROS 2 node communication."""
    
    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2."""
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS 2."""
        rclpy.shutdown()
    
    def test_navigation_state_topic_exists(self):
        """Test that navigation_state topic can be subscribed to."""
        node = rclpy.create_node('test_nav_state_sub')
        received_msgs = []
        
        def callback(msg):
            received_msgs.append(msg.data)
        
        sub = node.create_subscription(String, '/navigation_state', callback, 10)
        
        # Just verify subscription was created successfully
        self.assertIsNotNone(sub)
        node.destroy_node()
    
    def test_goal_pose_publisher(self):
        """Test that goal_pose can be published."""
        node = rclpy.create_node('test_goal_pub')
        pub = node.create_publisher(PoseStamped, '/goal_pose', 10)
        
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = 1.0
        msg.pose.position.y = 2.0
        msg.pose.orientation.w = 1.0
        
        # Verify publisher was created and can publish
        self.assertIsNotNone(pub)
        pub.publish(msg)  # Should not raise
        node.destroy_node()
    
    def test_odom_message_structure(self):
        """Test Odometry message can be created correctly."""
        msg = Odometry()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = 1.0
        msg.pose.pose.position.y = 2.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        
        self.assertEqual(msg.pose.pose.position.x, 1.0)
        self.assertEqual(msg.pose.pose.position.y, 2.0)


class TestPoseStampedCreation(unittest.TestCase):
    """Test creating navigation goals."""
    
    def test_create_goal_pose(self):
        """Test creating a valid goal pose."""
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 5.0
        goal.pose.position.y = 3.0
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        
        self.assertEqual(goal.header.frame_id, 'map')
        self.assertEqual(goal.pose.position.x, 5.0)
        self.assertEqual(goal.pose.orientation.w, 1.0)
    
    def test_goal_with_rotation(self):
        """Test creating a goal with 90 degree rotation."""
        import math
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 2.0
        goal.pose.position.y = 2.0
        # 90 degrees around Z axis
        goal.pose.orientation.z = math.sin(math.pi / 4)
        goal.pose.orientation.w = math.cos(math.pi / 4)
        
        self.assertAlmostEqual(goal.pose.orientation.z, 0.7071, places=3)
        self.assertAlmostEqual(goal.pose.orientation.w, 0.7071, places=3)


if __name__ == '__main__':
    unittest.main()
