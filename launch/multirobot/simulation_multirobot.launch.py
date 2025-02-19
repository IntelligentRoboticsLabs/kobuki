# Copyright (c) 2024 Intelligent Robotics Lab (URJC)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Modified by Juan Carlos Manzanares Serrano

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription
)
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import yaml


def spawn_robots(context):
    """Read the YAML configuration file and spawn the robots defined in it."""
    config_file = LaunchConfiguration('robots_config_file').perform(context)

    def convert_floats_to_strings(data):
        """
        Convert all float params in a dict to strings.

        This is required because all launch arguments must be strings.
        """
        if isinstance(data, dict):
            return {k: convert_floats_to_strings(v) for k, v in data.items()}
        elif isinstance(data, list):
            return [convert_floats_to_strings(i) for i in data]
        elif isinstance(data, float):
            return str(data)
        else:
            return data

    config_robots = yaml.safe_load(open(config_file, 'r'))
    config_robots = convert_floats_to_strings(config_robots)

    robot_actions = []
    for robot_args in config_robots['robots']:
        spawn_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('kobuki_description'),
                'launch/'), 'spawn.launch.py']),
            launch_arguments=robot_args.items()
        )
        robot_actions.append(spawn_robot)
    return robot_actions


def generate_launch_description():

    world_arg = DeclareLaunchArgument(
        'world', default_value=os.path.join(
            get_package_share_directory('aws_robomaker_small_house_world'),
            'worlds',
            'small_house.world'))

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to false to run gazebo headless',
    )

    robots_config_arg = DeclareLaunchArgument(
        'robots_config_file',
        default_value=os.path.join(
            get_package_share_directory('kobuki'),
            'config', 'multirobot',
            'multirobot_config.yaml'),
        description='YAML file with the configuration of the robots to be spawned',
    )

    # This argument is automatically forwarded to kobuki_description / spawn.launch.py
    declare_do_tf_remapping_arg = DeclareLaunchArgument(
        'do_tf_remapping',
        default_value='False',
        description='Whether to remap the tf topics to independent namespaces (/tf -> tf)',
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch',
                         'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -s ', LaunchConfiguration('world')]}.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch',
                         'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [' -g ']}.items(),
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    ld = LaunchDescription()
    ld.add_action(world_arg)
    ld.add_action(gui_arg)
    ld.add_action(robots_config_arg)
    ld.add_action(declare_do_tf_remapping_arg)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(OpaqueFunction(function=spawn_robots))

    return ld
