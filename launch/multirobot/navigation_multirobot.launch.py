# Copyright (c) 2018 Intel Corporation
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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EqualsSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import (
    Node,
    PushROSNamespace,
    SetParameter,
    SetRemap,
)
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml
import yaml


def launch_nodes(context):
    """Configure and launch all the navigation nodes."""
    kobuki_dir = get_package_share_directory('kobuki')
    launch_dir = os.path.join(kobuki_dir, 'launch', 'multirobot')

    lifecycle_nodes_navigation = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'velocity_smoother',
        'collision_monitor',
        'bt_navigator',
        'waypoint_follower',
        'docking_server',
    ]

    lifecycle_nodes_localization = [
        'map_server',
        'amcl'
    ]

    namespace = LaunchConfiguration('namespace')
    use_rviz = LaunchConfiguration('rviz')
    rviz_config_file = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    do_tf_remapping = LaunchConfiguration('do_tf_remapping')
    use_localization = LaunchConfiguration('use_localization')
    set_initial_pose = LaunchConfiguration('set_initial_pose').perform(context)
    set_initial_pose = set_initial_pose == 'True' or set_initial_pose == 'true'
    robots_config_file = LaunchConfiguration('robots_config_file').perform(context)

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    # remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    # Update(butakus): This is controlled by the do_tf_remapping launch argument.

    # Condition to check if a namespace is used
    is_empty_namespace = EqualsSubstitution(LaunchConfiguration('namespace'), '')

    # Only applies when `use_namespace` is True.
    # '<robot_namespace>' keyword shall be replaced by 'namespace' launch argument.
    # Config file 'nav2_multirobot_params_template.yaml' serves as a default & example.
    # User-defined config file should contain '<robot_namespace>' keyword for the replacements.
    # Notes:
    #   If a namespace is used, a trailing '/' is added to the namespace.
    #   If a namespace is not used (left empty), the tag <robot_namespace> is removed.
    params_file = ReplaceString(
        source_file=params_file,
        replacements={'<robot_namespace>': (namespace, '/')},
        condition=UnlessCondition(is_empty_namespace),
    )
    params_file = ReplaceString(
        source_file=params_file,
        replacements={'<robot_namespace>': ''},
        condition=IfCondition(is_empty_namespace),
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    navigation_nodes = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            PushROSNamespace(condition=UnlessCondition(is_empty_namespace), namespace=namespace),
            SetRemap(condition=IfCondition(do_tf_remapping), src='/tf', dst='tf'),
            SetRemap(condition=IfCondition(do_tf_remapping), src='/tf_static', dst='tf_static'),
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                parameters=[configured_params],
                remappings=[('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                parameters=[configured_params],
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[configured_params],
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[configured_params],
                remappings=[('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[configured_params],
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[configured_params],
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                parameters=[configured_params],
                remappings=[('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_monitor',
                output='screen',
                parameters=[configured_params],
            ),
            Node(
                package='opennav_docking',
                executable='opennav_docking',
                name='docking_server',
                output='screen',
                parameters=[configured_params],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'autostart': True}, {'node_names': lifecycle_nodes_navigation}],
            ),
        ],
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rviz_namespaced.launch.py')
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': namespace,
            'do_tf_remapping': do_tf_remapping,
            'rviz_config_file': rviz_config_file,
        }.items()
    )

    # Parse initial pose from multirobot setup config and build amcl params
    initial_pose = {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0,
        'yaw': 0.0
    }
    if set_initial_pose and os.path.isfile(robots_config_file):
        config_robots = yaml.safe_load(open(robots_config_file, 'r'))
        for robot_args in config_robots['robots']:
            if robot_args.get('namespace', '') == namespace.perform(context):
                initial_pose['x'] = robot_args.get('x', 0.0)
                initial_pose['y'] = robot_args.get('y', 0.0)
                initial_pose['z'] = robot_args.get('z', 0.0)
                initial_pose['yaw'] = robot_args.get('Y', 0.0)
                break
    amcl_extra_params = {
        'set_initial_pose': set_initial_pose,
        'initial_pose': initial_pose,
    }

    localization_nodes = GroupAction(
        condition=IfCondition(use_localization),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            PushROSNamespace(condition=UnlessCondition(is_empty_namespace), namespace=namespace),
            SetRemap(condition=IfCondition(do_tf_remapping), src='/tf', dst='tf'),
            SetRemap(condition=IfCondition(do_tf_remapping), src='/tf_static', dst='tf_static'),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[configured_params, {'yaml_filename': map_yaml_file}],
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[configured_params, amcl_extra_params],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{'autostart': True}, {'node_names': lifecycle_nodes_localization}],
            ),
        ],
    )

    return [navigation_nodes, rviz_cmd, localization_nodes]


def generate_launch_description():
    # Get the launch directory
    kobuki_dir = get_package_share_directory('kobuki')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='True', description='Whether to launch RVIZ')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(kobuki_dir, 'rviz', 'nav2_namespaced_view.rviz'),
        description='Full path to the RVIZ config file to use',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            kobuki_dir,
            'maps',
            'aws_house.yaml'),
        description='Full path to map yaml file to load'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            kobuki_dir, 'config', 'multirobot',
            'nav2_multirobot_params_template.yaml'
        ),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_do_tf_remapping_cmd = DeclareLaunchArgument(
        'do_tf_remapping',
        default_value='False',
        description='Whether to remap the tf topic to independent namespaces (/tf -> tf)',
    )

    declare_use_localization_cmd = DeclareLaunchArgument(
        'use_localization', default_value='True',
        description='Whether to enable localization or not'
    )

    declare_set_initial_pose_cmd = DeclareLaunchArgument(
        'set_initial_pose', default_value='True',
        description="""
            Set the initial_pose param in amcl.
            The pose value is read from the multirobot_config file.
        """
    )

    declare_robots_config_cmd = DeclareLaunchArgument(
        'robots_config_file',
        default_value=os.path.join(
            get_package_share_directory('kobuki'),
            'config', 'multirobot',
            'multirobot_config.yaml'),
        description="""
            YAML file with the configuration of the robots to be spawned.
            This file is only used to set the robot's initial pose.
        """,
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_do_tf_remapping_cmd)
    ld.add_action(declare_use_localization_cmd)
    ld.add_action(declare_set_initial_pose_cmd)
    ld.add_action(declare_robots_config_cmd)
    # Add the actions to launch all of the navigation nodes
    ld.add_action(OpaqueFunction(
        function=launch_nodes
    ))

    return ld
