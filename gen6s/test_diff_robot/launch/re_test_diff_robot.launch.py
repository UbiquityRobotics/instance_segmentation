import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    robotXacroName = 'test_diff_robot'

    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation time"),
        DeclareLaunchArgument("gui", default_value="true", description="Start RViz2 automatically"),
        DeclareLaunchArgument("world", default_value="empty.sdf", description="Specify the Gazebo world file"),
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

    # Package and file paths
    robot_package = "test_diff_robot"
    urdf_file_name = "diff_drive_robot.infloor.urdf.xacro"
    yaml_file_name = "my_controllers.yaml"

    # Paths
    pkg_share = get_package_share_directory(robot_package)
    urdf_path = os.path.join(pkg_share, "urdf", "robots", urdf_file_name)
    yaml_path = os.path.join(pkg_share, "config", yaml_file_name)
    rviz_config_file = PathJoinSubstitution([FindPackageShare(robot_package), "config", "robot_config.rviz"])
    
    # Expanded URDF file path
    expanded_urdf_path = os.path.join(pkg_share, "urdf", "robots", "expanded_robot_description.urdf")

    # Process the xacro file into an expanded URDF
    generate_expanded_urdf = ExecuteProcess(
        cmd=["xacro", urdf_path],
        output="screen",
        shell=False
    )

    # Load the robot description after generating the URDF
    robot_description_content = xacro.process_file(urdf_path).toxml()
    robot_description = {"robot_description": robot_description_content}

    # Control Node
    control_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[robot_description, {"yaml_description": yaml_path}],
                output="both",
                remappings=[("~/robot_description", "/robot_description")],
            )
        ]
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name='robot_state_publisher',
        output="both",
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description_content}]
    )

    # Spawning controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
        output="screen",
    )

    # Ensure `joint_state_broadcaster_spawner` starts after `diff_drive_controller_spawner`
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=diff_drive_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # Include Gazebo launch description
    gazebo_pkg_launch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    )
    gazebo_launch = IncludeLaunchDescription(
        gazebo_pkg_launch,
        launch_arguments={'gz_args': [f'-r -v 4 ', world_file]}.items()
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
    bridge_params = os.path.join(get_package_share_directory(robot_package), 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[bridge_params],
        output="screen"
    )

    # Launch description
    nodes = [
        LogInfo(msg=f"URDF Path: {urdf_path}"),
        LogInfo(msg=f"YAML Path: {yaml_path}"),
        generate_expanded_urdf,
        robot_state_publisher_node,
        control_node,
        diff_drive_controller_spawner,
        delay_joint_state_broadcaster,
        gazebo_launch,
        spawn_model_gazebo_node,
        rviz_node,
        ros_gz_bridge,
    ]

    return LaunchDescription(declared_arguments + nodes)
