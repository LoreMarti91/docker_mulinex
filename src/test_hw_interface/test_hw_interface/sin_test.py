#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pi3hat_moteus_int_msgs.msg import JointsCommand
import math

class SinTrajectoryNode(Node):
    def __init__(self, omega=1.0, amplitude=1.0, joint_name='joint1', publish_rate=100.0):
        super().__init__('sin_trajectory_node')

        # Store parameters from constructor
        self.omega = float(omega)
        self.amplitude = float(amplitude)
        self.joint_name = str(joint_name)
        self.publish_rate = float(publish_rate)

        # Publisher
        self.pub = self.create_publisher(JointsCommand, 'joint_controller/command', 10)

        # Timer
        period = 1.0 / self.publish_rate
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f"Sinusoidal trajectory node started: joint={self.joint_name}, omega={self.omega}, amplitude={self.amplitude}"
        )

    def timer_callback(self):
        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds * 1e-9

        # Compute sinusoidal trajectory
        position = self.amplitude * math.sin(self.omega * t)
        velocity = self.amplitude * self.omega * math.cos(self.omega * t)
        effort = 0.0

        msg = JointsCommand()
        msg.header.stamp = now.to_msg()
        msg.name = [self.joint_name]
        msg.position = [position]
        msg.velocity = [velocity]
        msg.effort = [effort]
        msg.kp_scale = [1.0]
        msg.kd_scale = [1.0]

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    # Example: modify trajectory here
    node = SinTrajectoryNode(omega=10.0, amplitude=0.5, joint_name='PORCODIO_JOINT', publish_rate=600.0)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()
