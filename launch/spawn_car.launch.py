#!/usr/bin/env python3
"""
Spawn a single 'car' robot into Gazebo Harmonic for the swarm_4df package.

Architecture (per-robot isolation strategy):
  - Generic link/joint names in the URDF (see basicmodel_urdf.xacro etc.)
    -- NOT baked-in namespaced names like the old "${ns}base_link".
  - robot_state_publisher's `frame_prefix` parameter applies the
    per-robot prefix to every published TF frame ("<name>/base_link",
    "<name>/lidar_link", ...).
  - Sensor/plugin topics in the xacro are left as plain, RELATIVE names
    ("cmd_vel", "joint_states", sensor name "lidar"/"imu" with no
    <topic> override) -- gz-sim auto-scopes those under
    /model/<robot_name>/... on its own, so every robot already gets a
    distinct Gazebo-side topic with zero xacro changes.
  - odom -> base_link TF comes from odom_tf_broadcaster.py (below), not
    from the DiffDrive plugin's own publish_odom_tf (disabled in
    gazebo_urdf.xacro). This node republishes that TF from the
    already-reliably-bridged <name>/odom topic -- using THAT message's
    own timestamp, never the node clock -- avoiding a version-sensitive
    gz.msgs.Pose_V <-> tf2_msgs/TFMessage bridge pairing.

This file spawns exactly ONE robot: robot_state_publisher, the Gazebo
entity, and its odom TF broadcaster. It does NOT bridge Gazebo topics
to ROS2 -- that's handled once,
for the whole swarm, by a single shared ros_gz_bridge process in
car_swarm_launch.py (built from ros_gz_bridge_swarm.yaml.template),
matching the original single-bridge-process design rather than spawning
N separate bridge nodes.

car_swarm_launch.py includes this file once per robot, staggered with a
TimerAction so simultaneous spawns don't collide in Gazebo.

No static TF is published here. Anchoring each robot's map frame to the
shared 'world' frame -- either via a known-offset static transform, or
via the swarm_4df_map_merge node -- happens once, at the swarm level,
in car_swarm_launch.py.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import xacro


def launch_setup(context, *args, **kwargs):
    name = LaunchConfiguration('name').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_share = get_package_share_directory('swarm_4df')
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')


    robot_description_xml = xacro.process_file(
        xacro_file,
        mappings={'name': name, 'prefix': name}
    ).toxml()
    robot_description = ParameterValue(robot_description_xml, value_type=str)

    # robot_state_publisher publishes robot_description as a *topic* too
    # (namespaced -> /<name>/robot_description), which the spawn node
    # below subscribes to. Both nodes can start concurrently since the
    # spawn node just waits for that topic.
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=name,
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'frame_prefix': f'{name}/',
            'use_sim_time': use_sim_time,
            
        }],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
        ],
    )

    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        name=f'spawn_{name}',
        output='screen',
        arguments=[
            '-name', name,
            '-topic', f'/{name}/robot_description',
            '-x', x, '-y', y, '-z', z, '-Y', yaw,
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Republishes <name>/odom -> odom_tf_broadcaster -> <name>/odom -> <name>/base_link
    # TF, timestamped from the Odometry message itself (see odom_tf_broadcaster.py
    # docstring for why this exists instead of bridging gz's own odom TF).
    odom_tf_node = Node(
        package='swarm_4df',
        executable='odom_tf_broadcaster',
        name='odom_tf_broadcaster',
        namespace=name,
        output='screen',
        parameters=[{'robot_name': name, 'use_sim_time': use_sim_time}],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
        ],
    )

    return [rsp_node, spawn_node, odom_tf_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('name', default_value='car1',
                               description='Robot name / ROS2 namespace'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.1'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])