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
    OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def start_description(context):

    description = []
    
    if LaunchConfiguration('xtion').perform(context) == 'true' or LaunchConfiguration('astra').perform(context) == 'true':
        camera = DeclareLaunchArgument('camera', default_value='true')
        description.append(camera)

    if LaunchConfiguration('lidar_a2').perform(context) == 'true' or LaunchConfiguration('lidar_s2').perform(context) == 'true':
        lidar = DeclareLaunchArgument('lidar', default_value='true')
        description.append(lidar)

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('kobuki_description'),
            'launch/'), 'kobuki_description.launch.py'])
    )

    return description + [robot_description]

def start_lidar(context):

    package_dir = get_package_share_directory('kobuki')
    filter_file = os.path.join(package_dir, 'config', 'footprint_filter.yaml')
        
    if LaunchConfiguration('lidar_a2').perform(context) == 'true':
        rplidar_cmd = Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'serial_port': '/dev/rplidar',
                'serial_baudrate': 115200,  # A1 / A2
                'frame_id': 'laser_link',
                'inverted': True,
                'namespace': LaunchConfiguration('namespace'),
                'angle_compensate': True,
            }],
        )

        laser_filter_cmd = Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            namespace=LaunchConfiguration('namespace'),
            parameters=[filter_file],
        )

        return [rplidar_cmd, laser_filter_cmd]

    if LaunchConfiguration('lidar_s2').perform(context) == 'true':
        rplidar__s2_cmd = Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'serial_port': '/dev/rplidar',
                'serial_baudrate': 1000000,  # S2
                'frame_id': 'laser_link',
                'inverted': True,
                'namespace': LaunchConfiguration('namespace'),
                'angle_compensate': True,
            }],
        )

        laser_filter_s2_cmd = Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            namespace=LaunchConfiguration('namespace'),
            parameters=[filter_file],
        )

        return [rplidar__s2_cmd, laser_filter_s2_cmd]

    return []

def start_camera(context):

    if LaunchConfiguration('xtion').perform(context) == 'true':
        xtion_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('openni2_camera'),
                'launch/'), 'camera_with_cloud.launch.py']),
        )

        return [xtion_cmd]

    if LaunchConfiguration('astra').perform(context) == 'true':
        astra_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('astra_camera'),
                'launch/'), 'astra_mini.launch.py']),
        )

        return [astra_cmd]

    return []

def generate_launch_description():
    package_dir = get_package_share_directory('kobuki')

    params_file = os.path.join(package_dir, 'config', 'kobuki_node_params.yaml')

    with open(params_file, 'r') as f:
        kobuki_params = yaml.safe_load(f)['kobuki_ros_node']['ros__parameters']

    declare_xtion_cmd = DeclareLaunchArgument(
        'xtion', default_value='false')

    declare_astra_cmd = DeclareLaunchArgument(
        'astra', default_value='false')

    declare_lidar_cmd = DeclareLaunchArgument(
        'lidar_a2', default_value='false')
    
    declare_lidar_s2_cmd = DeclareLaunchArgument(
        'lidar_s2', default_value='false')
    
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='')

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)

    kobuki_cmd = Node(
        package='kobuki_node',
        executable='kobuki_ros_node',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[kobuki_params],
        remappings=[
            ('/commands/velocity', '/cmd_vel')
        ]
    )

    ld.add_action(kobuki_cmd)
    ld.add_action(declare_xtion_cmd)
    ld.add_action(declare_astra_cmd)
    ld.add_action(declare_lidar_cmd)
    ld.add_action(declare_lidar_s2_cmd)
    ld.add_action(OpaqueFunction(function=start_description))
    ld.add_action(OpaqueFunction(function=start_lidar))
    ld.add_action(OpaqueFunction(function=start_camera))

    return ld
