#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64


class DynamicObstacleController(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_controller')

        # Use Gazebo simulation time (/clock)
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        # Publishes to the slider joint command topics of three dynamic boxes
        self.pubs = [
            self.create_publisher(Float64, '/model/dyn_box_01/joint/slider_joint/cmd_pos', 10),
            self.create_publisher(Float64, '/model/dyn_box_02/joint/slider_joint/cmd_pos', 10),
            self.create_publisher(Float64, '/model/dyn_box_03/joint/slider_joint/cmd_pos', 10),
        ]

        # When sim resets, /clock can jump backwards; we handle that cleanly.
        self.t0 = None

        self.timer = self.create_timer(0.05, self.update)  # 20 Hz
        self.get_logger().info('Dynamic obstacle controller started (use_sim_time:=True).')

    def update(self):
        now = self.get_clock().now()

        # Wait until /clock is active (sometimes starts at 0)
        t = now.nanoseconds * 1e-9 # Converts nanoseconds to seconds
        if t <= 0.0:
            return

        # Initialize / re-initialize reference time if needed
        if self.t0 is None:
            self.t0 = t

        # If simulation reset or time jumped backwards, restart phase smoothly
        # Incase gazebo crashes or relaunches
        if t < self.t0:
            self.t0 = t

        # Time since start (sim time)
        dt = t - self.t0

        # Sine-wave commands for each box, with different frequencies and amplitudes for the movement patterns
        cmds = [
            3.0 * math.sin(0.5 * dt),   # dyn_box_01: [-3, 3]
            2.3 * math.sin(0.7 * dt),   # dyn_box_02: [-2.5, 2.5]
            4.0 * math.sin(0.4 * dt),   # dyn_box_03: [-4, 4]
        ]

        # Pairs each publisher with its command and publishes
        for pub, val in zip(self.pubs, cmds):
            msg = Float64()
            msg.data = float(val)
            pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

