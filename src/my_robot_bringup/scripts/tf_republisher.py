#!/usr/bin/env python3
"""
TF Republisher - Fixes timestamp issues with static transforms in Gazebo simulation.

This node subscribes to /tf_static and republishes the transforms periodically
with the current simulation time, solving the "timestamp earlier than all data
in transform cache" errors that occur in slow simulations (e.g., VirtualBox).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import copy


class TFRepublisher(Node):
    def __init__(self):
        super().__init__('tf_republisher')
        
        # Store static transforms
        self.static_transforms = {}
        
        # QoS for static transforms (transient local)
        static_qos = QoSProfile(
            depth=100,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        
        # Subscribe to static transforms
        self.static_sub = self.create_subscription(
            TFMessage,
            '/tf_static',
            self.static_tf_callback,
            static_qos
        )
        
        # Publisher for dynamic TF (we'll republish statics as dynamic)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer to republish static transforms with current time
        # This ensures lidar_link transform is always available with recent timestamp
        self.timer = self.create_timer(0.05, self.republish_transforms)  # 20 Hz
        
        self.get_logger().info('TF Republisher started - fixing static transform timestamps')

    def static_tf_callback(self, msg: TFMessage):
        """Store static transforms for republishing."""
        for transform in msg.transforms:
            key = f"{transform.header.frame_id}_{transform.child_frame_id}"
            self.static_transforms[key] = transform
            self.get_logger().info(
                f'Captured static transform: {transform.header.frame_id} -> {transform.child_frame_id}',
                once=True
            )

    def republish_transforms(self):
        """Republish static transforms with current timestamp."""
        if not self.static_transforms:
            return
            
        current_time = self.get_clock().now().to_msg()
        
        transforms_to_publish = []
        for key, transform in self.static_transforms.items():
            # Create a copy with updated timestamp
            new_transform = TransformStamped()
            new_transform.header.stamp = current_time
            new_transform.header.frame_id = transform.header.frame_id
            new_transform.child_frame_id = transform.child_frame_id
            new_transform.transform = transform.transform
            transforms_to_publish.append(new_transform)
        
        # Publish all transforms
        for t in transforms_to_publish:
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = TFRepublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
