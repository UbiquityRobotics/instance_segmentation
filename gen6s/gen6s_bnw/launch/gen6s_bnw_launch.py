from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gen6s_bnw',
            executable='gen6s_bnw_node',
            name='gen6s_bnw_node',
            output='screen',
        )
    ])
