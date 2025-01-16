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

