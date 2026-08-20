#!/usr/bin/env python3
"""
Namespaced odom -> <name>/base_link TF broadcaster for the swarm_4df swarm.

Why this exists instead of bridging Gazebo's own odom TF into ROS:
gz-sim-diff-drive-system CAN publish odom->base_link TF itself
(publish_odom_tf), but getting that onto ROS2's /tf requires bridging
gz.msgs.Pose_V <-> tf2_msgs/msg/TFMessage, which is a version-sensitive
pairing that isn't reliably supported across ros_gz_bridge releases.
nav_msgs/msg/Odometry <-> gz.msgs.Odometry, by contrast, is a
rock-solid, well-supported bridge pairing -- so this node subscribes to
the (already reliably-bridged) <name>/odom topic and republishes the TF
itself. gazebo_urdf.xacro sets publish_odom_tf to false accordingly, so
there's exactly one TF source per robot, not two racing ones.

IMPORTANT (carried over from single-robot debugging):
Always use msg.header.stamp for the TF timestamp, NOT self.get_clock().now().
Using the wall/sim clock instead of the odometry message's own stamp caused
smeared SLAM maps previously with no obvious error - same risk applies here,
times three since each robot's slam_toolbox instance is timestamp-sensitive.

Frame names use "<name>/..." (e.g. "car1/odom", "car1/base_link") to match
robot_state_publisher's frame_prefix parameter exactly -- this node's
'robot_name' parameter must be set to the same value passed as 'prefix' to
the robot's xacro and as 'frame_prefix' to its robot_state_publisher (see
spawn_car_launch.py, which sets all three from one 'name' launch arg).
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTFBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')

        self.declare_parameter('robot_name', 'car1')
        self.robot_name = self.get_parameter('robot_name').value

        self.tf_broadcaster = TransformBroadcaster(self)

        # Topic is relative -> resolves to /<name>/odom when this node is
        # launched with namespace=<name>, matching the bridged
        # "<name>/odom" topic from ros_gz_bridge_swarm.yaml.template.
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        self.get_logger().info(
            f'Broadcasting TF for {self.robot_name}: '
            f'{self.robot_name}/odom -> {self.robot_name}/base_link'
        )

    def odom_callback(self, msg: Odometry):
        t = TransformStamped()

        # Use the odometry message's own stamp - do not substitute the node clock.
        t.header.stamp = msg.header.stamp
        # Namespaced odom frame - each robot's odometry is relative to its OWN
        # spawn point, so this must NOT be a shared "odom" frame across robots.
        # Matches robot_state_publisher's frame_prefix output exactly -- do
        # not change this to "${ns}odom" (no slash) or any other variant.
        t.header.frame_id = f'{self.robot_name}/odom'
        t.child_frame_id = f'{self.robot_name}/base_link'

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()