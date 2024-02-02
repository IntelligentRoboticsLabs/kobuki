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
from os import environ, pathsep

from ament_index_python.packages import (get_package_prefix,
                                         get_package_share_directory)

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import launch_ros.descriptions
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_model_paths(packages_names):
    model_paths = ''
    for package_name in packages_names:
        if model_paths != '':
            model_paths += pathsep

        package_path = get_package_prefix(package_name)
        model_path = os.path.join(package_path, 'share')

        model_paths += model_path

    return model_paths


def get_resource_paths(packages_names):
    resource_paths = ''
    for package_name in packages_names:
        if resource_paths != '':
            resource_paths += pathsep

        package_path = get_package_prefix(package_name)
        resource_paths += package_path

    return resource_paths


def generate_launch_description():

    world = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('R')
    pitch = LaunchConfiguration('P')
    yaw = LaunchConfiguration('Y')

    declare_world_cmd = DeclareLaunchArgument(
        'world', default_value=os.path.join(
            get_package_share_directory('aws_robomaker_small_house_world'),
            'worlds',
            'small_house.world'))

    declare_x_cmd = DeclareLaunchArgument(
        'x', default_value='0.0'
    )

    declare_y_cmd = DeclareLaunchArgument(
        'y', default_value='0.0'
    )

    declare_z_cmd = DeclareLaunchArgument(
        'z', default_value='0.0'
    )

    declare_roll_cmd = DeclareLaunchArgument(
        'R', default_value='0.0'
    )

    declare_pitch_cmd = DeclareLaunchArgument(
        'P', default_value='0.0'
    )

    declare_yaw_cmd = DeclareLaunchArgument(
        'Y', default_value='0.0'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'),
            'launch', 'gazebo.launch.py')]),
    )

    kobuki_dir = get_package_share_directory('kobuki_description')

    urdf_xacro_file = os.path.join(kobuki_dir, 'urdf', 'kobuki_hexagons_asus_xtion_pro_sim.urdf.xacro')

    model_name = DeclareLaunchArgument(
        'model_name', default_value='robot',
        description='Gazebo model name'
    )

    # Robot description
    robot_model = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': launch_ros.descriptions.ParameterValue(
                Command(['xacro ', urdf_xacro_file]), value_type=str),
            'use_sim_time': True
        }])

    # TF Tree
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    robot_entity_cmd = Node(package='gazebo_ros', executable='spawn_entity.py',
                            arguments=['-topic', 'robot_description',
                                       '-entity',
                                       LaunchConfiguration('model_name'),
                                       '-x', x,
                                       '-y', y,
                                       '-z', z,
                                       '-R', roll,
                                       '-P', pitch,
                                       '-Y', yaw,
                                       ],
                            output='screen')

    world_entity_cmd = Node(package='gazebo_ros', executable='spawn_entity.py',
                            arguments=['-entity',
                                       'world',
                                       '-file',
                                       world
                                       ],
                            output='screen')

    tf_footprint2base_cmd = Node(package='tf2_ros',
                                 executable='static_transform_publisher',
                                 output='screen',
                                 arguments=['0.0', '0.0', '0.0',
                                            '0.0', '0.0', '0.0',
                                            'base_link',
                                            'base_footprint'])
    
    fake_bumper_cmd = Node(package='kobuki', executable='fake_bumer_node', output='screen')

    ld = LaunchDescription()
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)
    ld.add_action(declare_z_cmd)
    ld.add_action(declare_roll_cmd)
    ld.add_action(declare_pitch_cmd)
    ld.add_action(declare_yaw_cmd)
    ld.add_action(model_name)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(robot_model)
    ld.add_action(tf_footprint2base_cmd)
    ld.add_action(robot_entity_cmd)
    ld.add_action(world_entity_cmd)
    ld.add_action(fake_bumper_cmd)

    packages = ['kobuki_description']
    model_path = get_model_paths(packages)
    resource_path = get_resource_paths(packages)

    if 'GAZEBO_MODEL_PATH' in environ:
        model_path += pathsep + environ['GAZEBO_MODEL_PATH']

    if 'GAZEBO_RESOURCE_PATH' in environ:
        resource_path += pathsep + environ['GAZEBO_RESOURCE_PATH']

    ld.add_action(SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path))
    ld.add_action(gazebo)

    return ld
