#!/usr/bin/env python3
"""
Activates the SLAM Toolbox lifecycle node after a delay.
Used to automatically start SLAM when launched.
"""

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
import time


class SlamActivator(Node):
    def __init__(self):
        super().__init__('slam_activator')
        
        self.get_logger().info('Waiting for SLAM Toolbox to be ready...')
        
        # Wait for service to be available
        self.change_state_client = self.create_client(
            ChangeState, '/slam_toolbox/change_state'
        )
        
        # Wait for service
        while not self.change_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /slam_toolbox/change_state service...')
        
        # Give SLAM time to initialize
        time.sleep(2.0)
        
        # Configure
        self.call_transition(Transition.TRANSITION_CONFIGURE)
        time.sleep(1.0)
        
        # Activate
        self.call_transition(Transition.TRANSITION_ACTIVATE)
        
        self.get_logger().info('SLAM Toolbox activated successfully!')
        
    # Calls the change_state service with the given transition ID    
    def call_transition(self, transition_id):
        request = ChangeState.Request()
        request.transition.id = transition_id
        
        future = self.change_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is not None:
            self.get_logger().info(f'Transition {transition_id} successful')
        else:
            self.get_logger().error(f'Transition {transition_id} failed')


def main(args=None):
    rclpy.init(args=args)
    node = SlamActivator()
    # Node job is done, shut down
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
