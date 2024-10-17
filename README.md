# Kobuki from Intelligent Robotics Lab using ROS 2

![distro](https://img.shields.io/badge/Ubuntu%2024-Noble%20Numbat-orange)
![distro](https://img.shields.io/badge/ROS2-Jazzy-blue)
[![jazzy-devel](https://github.com/IntelligentRoboticsLabs/kobuki/actions/workflows/jazzy-devel.yaml/badge.svg?branch=jazzy-devel)](https://github.com/IntelligentRoboticsLabs/kobuki/actions/workflows/jazzy-devel.yaml)

This project contains the launchers to run the [Turtlebot2 Kobuki](https://github.com/kobuki-base), both in simulated running different Gazebo worlds, as in the real robot using its drivers.

- [Installation on your own computer](#installation-on-your-own-computer)  
- [Run the robot in ROS 2](#run-the-robot-in-ros-2)  
   - [Run Gazebo](#run-gazebo)  
   - [Run a real kobuki](#run-a-real-kobuki)
- [Run Navigation in ROS 2](#run-navigation-in-ros-2)
- [About](#about)  


# Installation on your own computer
You need to have previously installed ROS2. Please follow this [guide](https://docs.ros.org/en/jazzy/Installation.html) if you don't have it.
```bash
source /opt/ros/jazzy/setup.bash
```

Clone the repository to your workspace:
```bash
cd <ros2-workspace>/src
git clone https://github.com/IntelligentRoboticsLabs/kobuki.git
```

Prepare your thirparty repos:
```bash
sudo apt update && sudo apt install ros-dev-tools -y
cd <ros2-workspace>/src/
vcs import < kobuki/thirdparty.repos
```
*Please make sure that this last command has not failed. If this happens, run it again.*

### Install libusb, libftdi & libuvc
```bash
sudo apt install libusb-1.0-0-dev libftdi1-dev libuvc-dev
```

### Install udev rules from astra camera, kobuki and rplidar
When you connect a piece of hardware to your pc, it assigns `/dev/ttyUSB*` to it. This will not have the necessary read/write permissions, so we will not be able to use it correctly. The solution is to set up some udev rules that creates a symlink with another name (example: `/dev/ttyUSB0` -> `/dev/kobuki`) and grants it the necessary permissions.
```bash
cd <ros2-workspace>
sudo cp src/ThirdParty/ros_astra_camera/astra_camera/scripts/56-orbbec-usb.rules /etc/udev/rules.d/
sudo cp src/ThirdParty/rplidar_ros/scripts/rplidar.rules /etc/udev/rules.d/
sudo cp src/ThirdParty/kobuki_ros/60-kobuki.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Building project
```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install 
```

>  If your terminal has crashed or closed while compiling, please try compiling your packages as follows `colcon build --symlink-install --parallel-workers 1` or do so by selecting the package that failed `colcon build --symlink-install --parallel-workers 1 --packages-select <package>`
> 
> Also, if you want to prevent it from recompiling that package, add a `COLCON_IGNORE` inside the package

# Run the robot in ROS 2
## Run Gazebo
You can launch the simulator as follows:
```bash
ros2 launch kobuki simulation.launch.py
```
Or you can add the path of your world to the world parameter like this:
```bash
ros2 launch kobuki simulation.launch.py world:=install/aws_robomaker_small_warehouse_world/share/aws_robomaker_small_warehouse_world/worlds/small_warehouse/small_warehouse.world
``` 

## Run a real kobuki
Run the kobuki drivers:

```bash
ros2 launch kobuki kobuki.launch.py
``` 

If you want to use a lidar or camera, you have to set the following parameters to true:
```bash
ros2 launch kobuki kobuki.launch.py lidar_a2:=true
ros2 launch kobuki kobuki.launch.py lidar_s2:=true
ros2 launch kobuki kobuki.launch.py xtion:=true
ros2 launch kobuki kobuki.launch.py astra:=true
``` 

# Run Navigation in ROS 2

You can use [Nav2](https://navigation.ros.org/) using robot with this launcher:

```bash
ros2 launch kobuki navigation.launch.py map:=<path-to-map>
``` 

or this other command if you need to navigate in the simulator
```bash
ros2 launch kobuki navigation_sim.launch.py
```

If you want to use another map, you have to put the route in the map parameter


# About

This is a project made by the [Intelligent Robotics Lab](https://intelligentroboticslab.gsyc.urjc.es/), a research group from the [Universidad Rey Juan Carlos](https://www.urjc.es/).
Copyright &copy; 2024.

Maintainers:

* [Juan Carlos Manzanares](https://github.com/Juancams)
