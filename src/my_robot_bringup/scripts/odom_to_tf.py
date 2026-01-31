#!/usr/bin/env python3
"""
Broadcasts odom -> base_footprint TF from /odom topic.
This is needed when the Gazebo bridge doesn't auto-publish this TF.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        
        # Use reliable QoS to match Gazebo bridge output
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos
        )
        
        self.get_logger().info('OdomToTF node started - broadcasting odom -> base_footprint')

    def odom_callback(self, msg: Odometry):
        t = TransformStamped()
        
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        
        t.transform.rotation = msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
