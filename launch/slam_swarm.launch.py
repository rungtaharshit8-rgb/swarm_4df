#!/usr/bin/env python3
"""
Per-robot SLAM for the swarm_4df car swarm.

Each robot runs its own namespaced async_slam_toolbox_node, producing its
own <name>/map. Each robot ALSO gets its own nav2_lifecycle_manager
instance, scoped to that robot's namespace and managing just that one
node (node_names=['slam_toolbox']) with autostart:=true.

This replaces the manual EmitEvent / RegisterEventHandler staggered
configure-then-activate choreography used in earlier versions of this
launch file: nav2_lifecycle_manager already knows how to walk a lifecycle
node through unconfigured -> inactive -> active on its own, and having
one manager per namespace means every robot's SLAM instance is
independently and automatically brought up -- no manual timers, no
cross-robot ordering assumptions.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory


ROBOT_NAMES = ['car1', 'car2', 'car3']


def generate_launch_description():
    pkg_share = get_package_share_directory('swarm_4df')
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_params_swarm.yaml')

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    ld = LaunchDescription([use_sim_time_arg])

    for name in ROBOT_NAMES:
        # base_frame/odom_frame/map_frame are overridden per robot on top
        # of the shared yaml, since slam_toolbox does NOT auto-namespace
        # these string parameters the way ROS2 auto-namespaces topics --
        # without this override every instance looks for a generic
        # "base_link"/"odom" frame that never matches the actual
        # "<name>/base_link"/"<name>/odom" frames.
        #
        # scan_topic is NOT overridden here: it's left as the relative
        # "lidar" from slam_params_swarm.yaml, and since this node is
        # itself launched inside namespace=name, ROS2 resolves that to
        # "/<name>/lidar" on its own -- which is also the real topic name
        # ros_gz_bridge_swarm.yaml.template bridges to (not "/scan").
        slam_node = LifecycleNode(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace=name,
            output='screen',
            parameters=[
                slam_params_file,
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'base_frame': f'{name}/base_link',
                    'odom_frame': f'{name}/odom',
                    'map_frame': f'{name}/map',
                },
            ],
        )
        ld.add_action(slam_node)

        ld.add_action(
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name=f'lifecycle_manager_slam_{name}',
                namespace=name,
                output='screen',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'autostart': True,
                    'node_names': ['slam_toolbox'],
                    'bond_timeout': 0.0,  # no timeout, since slam_toolbox is not a critical node
                }],
            )
        )

    return ld