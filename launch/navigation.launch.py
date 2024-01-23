# Copyright (c) 2023 José Miguel Guerrero Hernández
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

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_pal.include_utils import include_launch_py_description
from launch_ros.actions import SetRemap
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_dir = get_package_share_directory('kobuki')
    nav2_dir = get_package_share_directory('nav2_bringup')

    sim_time_arg = DeclareLaunchArgument(
      'use_sim_time', default_value='True')

    rviz_arg = DeclareLaunchArgument(
      'rviz', default_value='True')
    
    map_arg = DeclareLaunchArgument(
        'map', default_value=os.path.join(
        package_dir,
        'maps',
        'GrannieAnnie.yaml')
        )
    
    nav_params_arg = DeclareLaunchArgument(
        'params', default_value=os.path.join(
        package_dir,
        'config',
        'kobuki_nav_params.yaml')
        )

    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            # 'use_sim_time': LaunchConfiguration('use_sim_time'),
            # 'rviz': LaunchConfiguration('rviz'),
            'map': LaunchConfiguration('map'),
            # 'params': LaunchConfiguration('params')
        }.items()
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(nav_params_arg)
    ld.add_action(rviz_arg)
    ld.add_action(sim_time_arg)
    ld.add_action(map_arg)
    ld.add_action(nav2_cmd)

    return ld