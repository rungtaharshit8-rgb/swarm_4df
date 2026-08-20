#!/usr/bin/env python3
"""
Spawns the swarm_4df car swarm (car1, car2, car3) in Gazebo Harmonic, each
robot in its own ROS2 namespace with a fully isolated URDF/TF/topic stack
(generic link names + frame_prefix + ${prefix} topics -- see
spawn_car_launch.py).

Map-frame alignment strategy -- choose one via the 'static_map_tf' arg:

  static_map_tf:=true  (default)
      world -> <name>/map is published as a STATIC transform, at each
      robot's KNOWN spawn offset (the same x/y values used to spawn it
      below). Deterministic and zero-latency, but only correct because
      the offsets are hardcoded -- exactly the "known spawn offsets"
      setup this mode is meant for.

  static_map_tf:=false
      The custom swarm_4df_map_merge node (ORB feature matching between
      every robot's live SLAM map) discovers each robot's relative
      transform and publishes world -> <name>/map itself, continuously.
      No prior knowledge of spawn poses required.

Odom-frame bridging -- 'static_odom_tf' arg:

  static_odom_tf:=true  (default)
      <name>/map -> <name>/odom is published as a static IDENTITY
      transform. This is a legitimate interim placeholder, not a hack:
      map and odom coincide at spawn by SLAM convention, before any
      drift correction exists. It unblocks a single connected
      world -> <name>/map -> <name>/odom -> <name>/base_link TF tree
      immediately, independent of whether slam_toolbox is actually
      receiving scans yet.

  static_odom_tf:=false
      slam_toolbox owns this link exclusively, publishing real
      scan-matched corrections. MUST be false whenever slam_swarm.launch.py
      is running alongside this -- two broadcasters publishing the same
      parent (map -> odom) causes TF conflicts/jitter, not redundancy.

ROS <-> Gazebo bridging: ONE shared ros_gz_bridge process for the whole
swarm (not one per robot), built by patching
config/ros_gz_bridge_swarm.yaml with each robot's name and
concatenating the results into a single temp file -- the same
single-process design as the original slam_swarm_launch.py, just
generated instead of hand-duplicated per robot.

Run car_swarm_launch.py, then slam_swarm_launch.py (with
static_odom_tf:=false once lidar/SLAM is confirmed healthy), then
(optionally) nav2_swarm_launch.py, matching the layering already used
elsewhere in this repo (world -> spawn -> SLAM -> nav).
"""

import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory



ROBOTS = [
    {'name': 'car1', 'x': '1.0', 'y': '0.0', 'z': '0.1', 'yaw': '0.0'},
    {'name': 'car2', 'x': '1.0', 'y': '1.0', 'z': '0.1', 'yaw': '0.0'},
    {'name': 'car3', 'x': '1.0', 'y': '-1.0', 'z': '0.1', 'yaw': '0.0'},
]


def build_gz_bridge_node(context, *args, **kwargs):
    """Patch ros_gz_bridge_swarm.yaml.template once per robot and
    concatenate the results into one temp file for a single shared
    parameter_bridge process (see module docstring)."""
    import tempfile
    pkg_share = get_package_share_directory('swarm_4df')
    template_path = os.path.join(pkg_share, 'config', 'ros_gz_bridge_swarm.yaml')

    with open(template_path, 'r') as f:
        template_text = f.read()

    combined_text = '\n'.join(
        template_text.replace('__ROBOT_NAME__', robot['name']) for robot in ROBOTS
    )

    tmp_bridge = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
    tmp_bridge.write(combined_text)
    tmp_bridge.close()

    use_sim_time = LaunchConfiguration('use_sim_time')
    return [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge_swarm',
            output='screen',
            arguments=['--ros-args', '-p', f'config_file:={tmp_bridge.name}'],
            parameters=[{'use_sim_time': use_sim_time}],
        )
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('swarm_4df')
    world_file = os.path.join(pkg_share, 'worlds', 'multi_room.sdf')

    # --- RESTORED: this was missing, but is referenced below via
    # LaunchConfiguration('static_map_tf') -- undeclared configs raise
    # an error at launch time, not a silent no-op. ---
    static_map_tf_arg = DeclareLaunchArgument(
        'static_map_tf', default_value='false',
        description='true: static TF at known spawn offsets. '
                    'false: use the swarm_4df_map_merge node instead.'
    )

    static_odom_tf_arg = DeclareLaunchArgument(
        'static_odom_tf', default_value='false',
        description='true: bridge <name>/map -> <name>/odom with a static '
                'identity transform (interim, until slam_toolbox is '
                'confirmed publishing real corrections on this link). '
                'false: let slam_toolbox own this link exclusively.'
    )

    # --- RESTORED: also missing, also referenced below. ---
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    ld = LaunchDescription()
    ld.add_action(static_map_tf_arg)
    ld.add_action(static_odom_tf_arg)
    ld.add_action(use_sim_time_arg)

    # --- Start Gazebo Harmonic ---
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'),
                             'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': f'-r {world_file}'}.items()
        )
    )

    # Shared simulation clock -- separate from the per-robot bridge below,
    # since /clock is global regardless of how many robots exist.
    ld.add_action(
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            output='screen',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        )
    )

    # One shared ros_gz_bridge process for cmd_vel/odom/lidar/imu/
    # joint_states/tf across all 3 robots (see build_gz_bridge_node above).
    ld.add_action(OpaqueFunction(function=build_gz_bridge_node))

    # --- Spawn each robot (staggered so simultaneous spawns don't collide) ---
    for i, robot in enumerate(ROBOTS):
        ld.add_action(
            TimerAction(
                period=2.0 + i * 2.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(pkg_share, 'launch', 'spawn_car.launch.py')
                        ),
                        launch_arguments={
                            'name': robot['name'],
                            'x': robot['x'],
                            'y': robot['y'],
                            'z': robot['z'],
                            'yaw': robot['yaw'],
                            'use_sim_time': LaunchConfiguration('use_sim_time'),
                        }.items()
                    )
                ]
            )
        )

    # --- Map-frame anchoring, option 1: static TF at known offsets ---
    # slam_toolbox publishes <name>/map -> <name>/odom -> <name>/base_link
    # itself (map is the parent, per standard SLAM convention), so this
    # static anchor attaches at <name>/map, NOT <name>/odom.
    for robot in ROBOTS:
        ld.add_action(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'world_to_{robot["name"]}_map',
                output='screen',
                condition=IfCondition(LaunchConfiguration('static_map_tf')),
                arguments=[
                    '--x', robot['x'], '--y', robot['y'], '--z', '0',
                    '--yaw', robot['yaw'],
                    '--frame-id', 'world',
                    '--child-frame-id', f'{robot["name"]}/map',
                ],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            )
        )

    # --- RESTORED: this loop never made it in last time. This is the
    # actual point of static_odom_tf -- without it the arg is declared
    # but does nothing. Static identity <name>/map -> <name>/odom,
    # legitimate because they coincide at spawn before any SLAM drift
    # correction exists. Disable with static_odom_tf:=false once
    # slam_toolbox is confirmed publishing real corrections here. ---
    for robot in ROBOTS:
        ld.add_action(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'{robot["name"]}_map_to_odom_identity',
                output='screen',
                condition=IfCondition(LaunchConfiguration('static_odom_tf')),
                arguments=[
                    '--x', '0', '--y', '0', '--z', '0',
                    '--frame-id', f'{robot["name"]}/map',
                    '--child-frame-id', f'{robot["name"]}/odom',
                ],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            )
        )

    ld.add_action(
        Node(
            package='swarm_4df_map_merge',
            executable='map_merge',
            name='multi_robot_map_merger',
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('static_map_tf')),
            parameters=[{
                'robot_names': [r['name'] for r in ROBOTS],
                'match_confidence_threshold': 0.3,
                'min_map_size': 60000,
                'map_publish_frequency': 1.0,
                'tf_publish_frequency': 20.0,
                'visualize': False,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        )
    )

    return ld