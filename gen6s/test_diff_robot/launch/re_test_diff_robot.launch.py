import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():

    robotXacroName = 'test_diff_robot'

    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time"
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file"
        )
    ]

    # Initialize arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")

    # Package and file paths
    robot_package = "test_diff_robot"
    # urdf_file_name = "test_diff_robot.urdf.xacro"
    urdf_file_name = "diff_drive_robot.urdf"

    # yaml_file_name = "diff_drive_controller.yaml"
    yaml_file_name = "my_controllers.yaml"

    # Paths
    pkg_share = FindPackageShare(robot_package).find(robot_package)
    urdf_path = os.path.join(pkg_share, "urdf", "robots", urdf_file_name)
    yaml_path = os.path.join(pkg_share, "config", yaml_file_name)
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(robot_package), "config", "robot_config.rviz"]
    )



    # # Get URDF via xacro
    # robot_description_content = xacro.process_file(urdf_path).toxml()
    # robot_description = {"robot_description": robot_description_content}



    #alternatively try to use the xacro file directly
    # Add this path for the expanded URDF
    expanded_urdf_path = os.path.join(pkg_share, "urdf", "robots", "expanded_robot_description.urdf")
    # expanded_urdf_path = os.path.join(pkg_share, "urdf", "robots", "diff_drive_robot.urdf")


    # Process the xacro file into an expanded URDF
    generate_expanded_urdf = ExecuteProcess(
        cmd=[
            "xacro", urdf_path, ">", expanded_urdf_path
        ],
        shell=True,  # Allow shell commands for the redirection
        output="screen",
    )

    # Get URDF via xacro
    robot_description_content = xacro.process_file(expanded_urdf_path).toxml()
    robot_description = {"robot_description": robot_description_content}

    # Nodes
    control_node = TimerAction(
    period=10.0,  # Delay in seconds
    actions=[
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, {"yaml_description": yaml_path}],
            output="both",
            remappings=[
                ("~/robot_description", "/robot_description"),
            ],
        )
    ]
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name='robot_state_publisher',
        output="both",
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content}]
    )
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

    # cmd_vel publisher node
    # cmd_vel_publisher_node = Node(
    #     package='test_diff_robot',  # 
    #     executable='cmd_vel_publisher',  # Replace with the name of the Python script
    #     output='screen',
    # )

    # Delay joint state broadcaster start after controller
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=diff_drive_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # Launch Gazebo
    gazebo_process = ExecuteProcess(
        cmd=["gz", "sim", "empty.sdf"],
        output="screen",
    )

    # Delay URDF spawning in Gazebo
    spawn_urdf_in_gazebo = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "gz", "service", "-s", "/world/empty/create",
                    "--reqtype", "gz.msgs.EntityFactory",
                    "--reptype", "gz.msgs.Boolean",
                    "--timeout", "1000",
                    "--req", f'sdf_filename: "{expanded_urdf_path}", name: "urdf_model"'
                ],
                output="screen",
            )
        ],
    )

    # URDF spawning in Gazebo

    # spawnModelNode = Node(package='gazebo_ros', executable='spawn_entity.py',
    #                       arguments=['-topic','robot_description','-entity', robotXacroName],output='screen')

    # RViz2 Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
        output="screen",
    )

    bridge_params = os.path.join(get_package_share_directory(robot_package),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    # rviz_node = Node(
    # package="rviz2",
    # executable="rviz2",
    # name="rviz2",
    # output="screen",
    # )

    # Delay RViz2 start after joint state broadcaster
    # delay_rviz_after_joint_state_broadcaster = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[rviz_node],
    #     )
    # )

    # Create the launch description
    nodes = [
        LogInfo(msg=f"URDF Path: {urdf_path}"),
        LogInfo(msg=f"YAML Path: {yaml_path}"),
        DeclareLaunchArgument("use_sim_time", default_value="false", description="Use sim time if true"),
        generate_expanded_urdf,
        control_node,
        robot_state_publisher_node,
        diff_drive_controller_spawner,
        delay_joint_state_broadcaster,
        gazebo_process,
        spawn_urdf_in_gazebo,
        # spawnModelNode,
        rviz_node,
        ros_gz_bridge
        # delay_rviz_after_joint_state_broadcaster,
    ]

    return LaunchDescription(declared_arguments + nodes
                              )
