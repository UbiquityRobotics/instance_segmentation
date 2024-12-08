# Setup Guide

## Requirements
1. **Ros 2**(Jazzy has been used for the development, change jazzy to your distro in place of jazzy in code snippets )

2. **Webots**

Place the package inside the src folder of your Ros2 workspace

<your_workspace\src\intro_room>

Run these commands first in any terminal that you want to run Ros2 in

```
cd <your_workspace>
source /opt/ros/jazzy/setup.bash
colcon build
```
## Installing Webots

Follow the steps for installation of Webots
```
sudo apt-get install ros-jazzy-webots-ros2
```
Source the terminal

```
source /opt/ros/jazzy/setup.bash
```
Set the WEBOTS_HOME enviornment variable
```
export WEBOTS_HOME=/usr/local/webots
```
launch a demo 

```
ros2 launch webots_ros2_universal_robot multirobot_launch.py
```
If you get a prompt to install webots package, accept it.
# Running simulation with Webots

In the first terminal start the simulation with

```
ros2 launch intro_room robot_launch.py

```
If you get a prompt to install webots package, accept it.


In another terminal give the command with the /cmd_vel topic

```
ros2 topic pub /cmd_vel geometry_msgs/Twist  "linear: { x: 0.1 }"

```

the bot will move in a straight line until it hits the edges of the map.