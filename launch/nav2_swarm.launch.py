#!/usr/bin/env python3

import os
import tempfile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


ROBOT_NAMES = ['car1', 'car2', 'car3']


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('swarm_4df')
    nav2_bringup_launch = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
    template_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    with open(template_path, 'r') as f:
        template_text = f.read()

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    autostart = LaunchConfiguration('autostart').perform(context)

    actions = []
    for name in ROBOT_NAMES:
        patched_text = template_text.replace('__ROBOT_NAME__', name)

        tmp_params = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
        tmp_params.write(patched_text)
        tmp_params.close()

        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_bringup_launch),
                launch_arguments={
                    'namespace': f'/{name}',
                    'use_namespace': 'True',
                    'use_sim_time': use_sim_time,
                    'params_file': tmp_params.name,
                    'slam': 'True',
                    'use_localization': 'False',
                    'use_composition': 'False',
                    'autostart': autostart,
                }.items()
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
