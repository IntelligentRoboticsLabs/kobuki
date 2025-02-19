# Multi-Robot System Launch Files

This directory contains launch files to configure and launch a Multi-Robot system with the kobuki simulator in Gazebo and ROS 2.

## Run Gazebo

You can run the simulation with multiple robots as follows:
```bash
ros2 launch kobuki simulation_multirobot.launch.py
```

The specific configuration of the multi-robot system is specified in the [config/multirobot/multirobot_config.yaml](../../config/multirobot/multirobot_config.yaml). 
In that file it is possible to setup the number of robots, names, initial positions, and more. Refer to the yaml file to know more about this configuration.

Addiitonally, the launch arguments present in the regular `simulation.launch.py` (world, gui, etc.) are also available in the multirobot version.

### Namespaces and TF
To avoid name collisions, it is important to set a different namespace for each robot.
This namespace will not only modify the names of topics and nodes, but will also modify the TF tree, so the original frame `base_link` will become `<robot_namespace>/base_link`.
This is important to discern the origin frame of any message in the system.

### TF topic remapping
With the default configuration, all robots share a common, global, `/tf` topic. This can be ok in simulation, but can be a source of problems in real world systems (more info in [1], [2], [3]).

For this reason, the common approach in nav2 is to remap the `tf` topics to each robot's namespace.
You can have the same behaviour by setting the `do_tf_remapping` launch argument to `true`. That will effectively isolate the TF tree of each robot into their respective `<robot_namespace>/tf` topics.

```bash
ros2 launch kobuki simulation_multirobot.launch.py do_tf_remapping:=true
```

[1]: https://github.com/ros/geometry2/issues/32
[2]: https://github.com/ros/robot_state_publisher/pull/30
[3]: https://discourse.ros.org/t/tf-tree-in-a-multi-robot-setup-in-ros2/41426/5

## Run Navigation
To start the navigation with nav2, it is required to launch the `navigation_multirobot.launch.py` file independently for each robot, properly setting the `namespace` argument. 

```bash
ros2 launch kobuki navigation_multirobot.launch.py namespace:=<robot_namespace>
```

**Note**: Right now, running `nav2` with the `slam` option (running slam_toolbox) is not supported. If you need to create the map, build the map with a single robot and reuse the map file later with the multi-robot system.


### Launch Arguments:

The `navigation_multirobot.launch.py` file accepts several launch arguments to configure nav2. Below is a list of some of the most important:

- `namespace`: Top-level namespace for the robot. This argument is used to avoid name collisions between multiple robots.
- `rviz`: Whether to launch an RVIZ instance. Default is True. Set it to false if you want to use your own RVIZ setup.
- `map`: Full path to the map yaml file to load. Default is `kobuki/maps/aws_house.yaml`.
- `do_tf_remapping`: Whether to remap the tf topic to independent namespaces (/tf -> tf). Default is False.
- `use_localization`: Whether to enable localization (AMCL) or not. Default is True.
- `set_initial_pose`: Set the initial_pose param in amcl. By default (True), the pose value is read from the multirobot_config file. That means that there is no need to set the initial pose estimation on RVIZ for each robot, as i is taken automatically from the setup file. Set this argument to False if you want to set this manually or through your own external system.


### About the multi-robot templated configuration files
In order to properly set the parameters for the nav2 nodes and rviz, the multi-robot launch system takes the templated configuration files [nav2_multirobot_params_template.yaml](../../config/multirobot/nav2_multirobot_params_template.yaml) and [nav2_namespaced_view.rviz](../../rviz/nav2_namespaced_view.rviz). These configuration files are automatically modified by the launch system to replace the `<robot_namespace>` tags for the corresponding value for each robot. If you want to create your own configuration, please follow the same format (More info in the [launch file](navigation_multirobot.launch.py#L89)).



## Example

Here is an example of how to launch a (simulated) system with two kobuki robots:

1. Launch the simulation:

    ```bash
    # Terminal 1
    ros2 launch kobuki simulation_multirobot.launch.py
    ```

2. For each robot, launch nav2:

    ```bash
    # Terminal 2
    ros2 launch kobuki navigation_multirobot.launch.py namespace:=r1
    ```
    ```bash
    # Terminal 3
    ros2 launch kobuki navigation_multirobot.launch.py namespace:=r2
    ```

Two RVIZ windows will open (one for each robot) that can be used to manually set the navigation goal.

Feel free to adjust the [YAML configuration file](../../config/multirobot/multirobot_config.yaml) or to provide your own.
