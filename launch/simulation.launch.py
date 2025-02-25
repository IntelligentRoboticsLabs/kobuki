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
    IncludeLaunchDescription,
    SetEnvironmentVariable
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('kobuki_description'),
            'launch/'), 'spawn.launch.py']),
    )

    fake_bumper = Node(
        package='kobuki',
        executable='fake_bumer_node',
        name='fake_bumer_node',
        output='screen',
    )

    model_path = ''
    resource_path = ''
    pkg_path = get_package_share_directory('kobuki')
    model_path += os.path.join(pkg_path, 'models')
    resource_path += pkg_path + model_path

    if 'GZ_SIM_MODEL_PATH' in os.environ:
        model_path += os.pathsep+os.environ['GZ_SIM_MODEL_PATH']
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        resource_path += os.pathsep+os.environ['GZ_SIM_RESOURCE_PATH']

    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', model_path))
    ld.add_action(world_arg)
    ld.add_action(gui_arg)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(spawn_robot)
    ld.add_action(fake_bumper)

    return ld
