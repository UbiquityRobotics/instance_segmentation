# Ubiquity Robotics Gen6s Color Publisher
![OS](https://img.shields.io/ubuntu/v/ubuntu-wallpapers/noble)
![ROS_2](https://img.shields.io/ros/v/jazzy/rclcpp)


Use the following command to check the integrity of the urdf file for simulation in Gazebo sim
```
check_urdf test_diff_robot.urdf
```

Launch using 
```
ros2 launch test_diff_robot test_diff_robot.launch.py
```

Next, open RViz and set the fixed frame. 

1. Open RViz:
    ```
    rviz2
    ```
2. In the "Global Options" section, set the "Fixed Frame" to `base_link`.
3. Add necessary displays such as RobotModel, TF, and LaserScan to visualize the robot.


to test if the xacro format is correct run
xacro path_to_yout_file.xacro

xacro test_diff_robot.urdf.xacro > expanded_robot_description.urdf

ros2 run urdf_parser_py check_urdf expanded_robot_description.urdf

check_urdf expanded_robot_description.urdf

ros2 run rqt_image_view rqt_image_view

check the publishing to the cmd_vel topic
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5"

ros2 run joint_state_publisher_gui joint_state_publisher_gui


Add visualize lidar in the options of gazebo sim to see the lidar data. reload to see the /scan topic publishing
important installation
sudo apt install ros-humble-gz-ros2-control 