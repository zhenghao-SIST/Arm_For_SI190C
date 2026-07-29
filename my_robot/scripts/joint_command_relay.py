#!/usr/bin/env python3
"""
Joint Command Relay Node

Subscribes to /joint_states (e.g. from joint_state_publisher_gui)
and forwards position commands to individual joint command topics
that are bridged to Gazebo.

This allows the joint_state_publisher_gui (used by display.launch.py)
to control the robot arm in Gazebo.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class JointCommandRelay(Node):
    """Relays /joint_states to per-joint /cmd_pos topics for Gazebo bridge."""

    def __init__(self):
        super().__init__('joint_command_relay')

        # Map of joint names to publishers (lazily created)
        self.cmd_publishers: dict = {}

        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        self.get_logger().info(
            'Joint Command Relay started. '
            'Forwarding /joint_states to per-joint /cmd_pos topics.'
        )

    def get_publisher(self, joint_name: str):
        """Create or retrieve a publisher for a joint command topic."""
        if joint_name not in self.cmd_publishers:
            # Bridge maps: Gazebo /model/my/joint/<joint>/0/cmd_pos
            #          → ROS /joint/<joint>/cmd_pos (avoids token "0" issue)
            topic = f'/joint/{joint_name}/cmd_pos'
            self.cmd_publishers[joint_name] = self.create_publisher(
                Float64, topic, 10
            )
            self.get_logger().info(f'Created publisher: {topic}')
        return self.cmd_publishers[joint_name]

    def joint_state_callback(self, msg: JointState):
        """Forward position commands from /joint_states to individual joints."""
        for name, position in zip(msg.name, msg.position):
            pub = self.get_publisher(name)
            cmd_msg = Float64()
            cmd_msg.data = float(position)
            pub.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointCommandRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
