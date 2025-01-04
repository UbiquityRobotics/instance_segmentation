from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gen6s_color',
            executable='gen6s_color_node',
            name='gen6s_color_node',
            output='screen',
        ),
    ])
