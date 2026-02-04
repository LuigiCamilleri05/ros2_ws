#!/usr/bin/env python3
"""
Scan Timestamp Fixer - Fixes timestamp issues with LaserScan in Gazebo simulation.

Gazebo sensor data often arrives with old timestamps while TF has newer timestamps.
This node republishes scan data with the current simulation time, fixing the
"timestamp earlier than all data in transform cache" errors.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanTimestampFixer(Node):
    def __init__(self):
        super().__init__('scan_timestamp_fixer')
        
        # QoS for sensor data - match Gazebo bridge output
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT, # Best effort for sensor data
            durability=DurabilityPolicy.VOLATILE # Don't store messages for late subscribers
        )
        
        # QoS for republished data - reliable for Nav2
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE, # Keeps messages for Nav2
            durability=DurabilityPolicy.VOLATILE # Don't store messages for late subscribers
        )
        
        # Subscribe to original scan
        self.scan_sub = self.create_subscription(
            LaserScan, # Message type
            '/scan_raw',  # We'll remap /scan to /scan_raw in launch
            self.scan_callback,
            sensor_qos
        )
        
        # Republish with fixed timestamp
        self.scan_pub = self.create_publisher(
            LaserScan,
            '/scan',
            reliable_qos
        )
        
        self.get_logger().info('Scan Timestamp Fixer started - fixing LaserScan timestamps')

    def scan_callback(self, msg: LaserScan):
        """Republish scan with current timestamp."""
        # Update timestamp to current simulation time
        msg.header.stamp = self.get_clock().now().to_msg()
        self.scan_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanTimestampFixer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
