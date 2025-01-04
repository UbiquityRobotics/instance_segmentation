# Ubiquity Robotics Gen6s Color Publisher
![OS](https://img.shields.io/ubuntu/v/ubuntu-wallpapers/noble)
![ROS_2](https://img.shields.io/ros/v/jazzy/rclcpp)


when you change the package.xml

rosdep install -r --from-paths src -i -y --rosdistro jazzy

then 

colcon build --packages-select gen6s