import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('test_diff_robot')
    urdf_path = os.path.join(pkg_path, 'urdf','robots', 'robot.urdf.xacro')
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', urdf_path],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_path}]
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[{'robot_description': urdf_path}]
        )
    ])