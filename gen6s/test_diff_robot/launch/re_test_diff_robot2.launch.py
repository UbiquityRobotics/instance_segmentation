import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro
from launch.actions import OpaqueFunction



def generate_launch_description():
    robotXacroName = 'differential_drive_robot'

    # Package and file pathsz
    robot_package = "test_diff_robot"
    # urdf_file_name = "robot.xacro"
    urdf_file_name = "diff_drive_robot.infloor2.urdf.xacro"

    # urdf_file_name = "diff_drive_robot.infloor.urdf.xacro"
    yaml_file_name = "gz_bridge.yaml"

    # world file name
    world_file_name = "empty.world"

    pkg_share = get_package_share_directory(robot_package)
    world_path = os.path.join(pkg_share, "worlds", world_file_name)

    #debugging world path issue
    print(f"Computed world file path: {world_path}")


    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation time"),
        DeclareLaunchArgument("gui", default_value="true", description="Start RViz2 automatically"),
        # DeclareLaunchArgument("world", default_value="empty.sdf", description="Specify the Gazebo world file"),
        DeclareLaunchArgument("world", default_value=world_path, description="Specify the Gazebo world file"), #added the new world with configs
        DeclareLaunchArgument("x", default_value="0.0", description="Initial X position"),
        DeclareLaunchArgument("y", default_value="0.0", description="Initial Y position"),
        DeclareLaunchArgument("z", default_value="0.5", description="Initial Z position"),
        DeclareLaunchArgument("R", default_value="0.0", description="Initial Roll"),
        DeclareLaunchArgument("P", default_value="0.0", description="Initial Pitch"),
        DeclareLaunchArgument("Y", default_value="0.0", description="Initial Yaw"),
    ]

    

    # Retrieve launch configurations
    world_file = LaunchConfiguration('world')
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    roll, pitch, yaw = LaunchConfiguration('R'), LaunchConfiguration('P'), LaunchConfiguration('Y')
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")

    

    # Paths
    pkg_share = get_package_share_directory(robot_package)
    urdf_path = os.path.join(pkg_share, "urdf", "robots", urdf_file_name)
    yaml_path = os.path.join(pkg_share, "config", yaml_file_name)
    rviz_config_file = PathJoinSubstitution([FindPackageShare(robot_package), "config", "robot_config.rviz"])

    # Process the xacro file into URDF directly
    try:
        robot_description_content = xacro.process_file(urdf_path).toxml()
    except Exception as e:
        raise RuntimeError(f"Error processing xacro file {urdf_path}: {str(e)}")

    robot_description = {"robot_description": robot_description_content}

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name='robot_state_publisher',
        output="both",
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description_content}]
    )

    # Include Gazebo launch description
    gazebo_pkg_launch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    )
    gazebo_launch = IncludeLaunchDescription(
        gazebo_pkg_launch,
        # launch_arguments={'gz_args': [f'-r -v 4 ', world_file]}.items()
        # launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items()
        launch_arguments={'gz_args': ['-r -v4 ', world_file], 'on_exit_shutdown': 'true'}.items()
    )

    # Spawn model in Gazebo
    spawn_model_gazebo_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robotXacroName,
            '-string', robot_description_content,
            '-x', x,
            '-y', y,
            '-z', z,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw,
            '-allow_renaming', 'false'
        ],
        output='screen',
    )

    # RViz2 Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
        output="screen",
    )

    # Bridge for ROS2-Gazebo communication
    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args', '-p',
            f'config_file:={yaml_path}'
        ],
        output='screen'
    )

    # Bridge for the image node
    gz_image_bridge_node = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen'
    )

    # Launch description
    nodes = [
        LogInfo(msg=f"URDF Path: {urdf_path}"),
        LogInfo(msg=f"YAML Path: {yaml_path}"),
        LogInfo(msg=f"World Path: {world_file}"),
        robot_state_publisher_node,
        gazebo_launch,
        spawn_model_gazebo_node,
        rviz_node,
        gz_bridge_node,
        gz_image_bridge_node
    ]

    return LaunchDescription(declared_arguments + nodes)
