#!/usr/bin/env python3
"""
Camera Point Cloud Processor Node

This node processes the point cloud from the RGBD camera sensor,
fixing timestamps and frame IDs for compatibility with Nav2 costmaps.

Similar to scan_timestamp_fixer.py, this ensures that:
1. Timestamps are synchronized with ROS time (use_sim_time aware)
2. Frame IDs are correctly set for TF lookups
3. QoS settings match what Nav2 expects
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2


class CameraPointCloudProcessor(Node):
    def __init__(self):
        super().__init__('camera_pointcloud_processor')
        
        # Declare parameters
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('input_topic', '/camera/points')
        self.declare_parameter('output_topic', '/camera/points_processed')
        self.declare_parameter('frame_id', 'camera_link_optical')
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # QoS for sensor data - best effort to match Gazebo bridge
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # Don't retry failed messages
            durability=DurabilityPolicy.VOLATILE, # Don't store messages for late subscribers
            history=HistoryPolicy.KEEP_LAST, # Only keep recent messages
            depth=5 # Keep last 5 messages in queue
        )
        
        # QoS for output - reliable for Nav2 compatibility
        output_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, # Guarantee message delivery
            durability=DurabilityPolicy.VOLATILE, # Don't store messages for late subscribers
            history=HistoryPolicy.KEEP_LAST, # Only keep recent messages
            depth=5 # Keep last 5 messages in queue
        )
        
        # Subscribe to raw point cloud from Gazebo
        self.subscription = self.create_subscription(
            PointCloud2, # Message type (3d point cloud)
            input_topic, # /camera/points from Gazebo bridge
            self.pointcloud_callback, # Function to call when a message is received
            sensor_qos # QoS profile
        )
        
        # Publisher for processed point cloud
        self.publisher = self.create_publisher(
            PointCloud2,
            output_topic,
            output_qos
        )
        
        self.get_logger().info(
            f'Camera PointCloud Processor started: {input_topic} -> {output_topic}'
        )
        self.get_logger().info(f'Using frame_id: {self.frame_id}')
        
        # Statistics
        self.msg_count = 0
        self.last_log_time = self.get_clock().now()
    
    def pointcloud_callback(self, msg: PointCloud2):
        """Process incoming point cloud and republish with fixed headers."""
        # Create a copy of the message header
        processed_msg = PointCloud2()
        
        # Copy all point cloud data
        processed_msg.height = msg.height
        processed_msg.width = msg.width
        processed_msg.fields = msg.fields
        processed_msg.is_bigendian = msg.is_bigendian
        processed_msg.point_step = msg.point_step
        processed_msg.row_step = msg.row_step
        processed_msg.data = msg.data
        processed_msg.is_dense = msg.is_dense
        
        # Fix header with current time and correct frame (main purpose)
        processed_msg.header.stamp = self.get_clock().now().to_msg()
        processed_msg.header.frame_id = self.frame_id
        
        # Publish processed message
        self.publisher.publish(processed_msg)
        
        # Log statistics periodically
        self.msg_count += 1
        current_time = self.get_clock().now()
        elapsed = (current_time - self.last_log_time).nanoseconds / 1e9
        
        if elapsed >= 10.0:  # Log every 10 seconds
            rate = self.msg_count / elapsed
            self.get_logger().info(
                f'Processed {self.msg_count} point clouds ({rate:.1f} Hz)'
            )
            self.msg_count = 0
            self.last_log_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = CameraPointCloudProcessor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down camera pointcloud processor')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
