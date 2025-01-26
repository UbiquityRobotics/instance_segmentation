import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess, LogInfo, TimerAction
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import sys


import xacro


def generate_launch_description():


    robot_package = "test_diff_robot"
    # urdf_file_name = "test_diff_robot_main.urdf.xacro"  # Adjust based on your URDF filename
    urdf_file_name = "test_diff_robot.urdf.xacro"
    # urdf_file_name = "articubot.xacro"
    # urdf_file_name = "test_diff_robot.urdf.xacro"  # Adjust based on your URDF filename
    # urdf_file_name = "test_diff_robotarti.urdf.xacro"  # Adjust based on your URDF filename
    # urdf_path = PathJoinSubstitution([FindPackageShare(urdf_package), "urdf", urdf_file_name])
    # yaml_path = PathJoinSubstitution([FindPackageShare(urdf_package), "config", "diff_drive_controller.yaml"])

    # urdf_package = "test_diff_robot"
    yaml_file_name = "diff_drive_controller.yaml" 

    # Path to the RViz config file
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare(robot_package),
        "config",
        "robot_config.rviz"
    ])

    pkg_share_description = FindPackageShare(robot_package).find(robot_package)
    # default_urdf_model_path = PathJoinSubstitution(
    # urdf_path = PathJoinSubstitution(
    #     [pkg_share_description, 'urdf', urdf_file_name])
    urdf_dir = os.path.join(pkg_share_description, 'urdf')
    robots_dir = os.path.join(urdf_dir, 'robots')
    urdf_path = os.path.join(robots_dir, urdf_file_name)
    # yaml_path = PathJoinSubstitution(
    #     [pkg_share_description, 'config', yaml_file_name])
    
    robot_description_config = xacro.process_file(urdf_path) #use only for articubot 


    # Create a robot_state_publisher node
    use_sim_time = LaunchConfiguration('use_sim_time')
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )       

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    params = {'robot_description': robot_desc}

    robot_description_config = xacro.process_file(urdf_path)
    robot_description = {'robot_description': robot_description_config.toxml()}


    config_file_name = "diff_drive_controller.yaml"
    config_dir = os.path.join(pkg_share_description, 'config')
    config_path = os.path.join(config_dir, config_file_name)
    # yaml_path = PathJoinSubstitution(
    #     [pkg_share_description, 'config', yaml_file_name])

    #yaml description definition
    with open(config_path, 'r') as infp:
        config_desc = infp.read()

    params = {'yaml_description': config_desc}

    



    return LaunchDescription([

        LogInfo(
            msg=f"URDF Path: {urdf_path}",
        ),
        LogInfo(
            msg=f"YAML Path: {config_path}",
        ),

#-----------------------------------------------------------------articubot testing
        DeclareLaunchArgument(
                                    'use_sim_time',
                                    default_value='false',
                                    description='Use sim time if true'),

                                node_robot_state_publisher
                            ,
#---------------------------------------------------------------------------------
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     namespace='/robot_namespace',
        #     name='robot_state_publisher',
        #     output='screen',
        #     # parameters=[{'robot_description': Command(['xacro ', urdf_file])}] //use this when we use xacro file
        #     # parameters=[{'robot_description': urdf_file}],
        #     # parameters=[{"robot_description": urdf_path}, yaml_path],
        #     # parameters=[{"robot_description": urdf_file_name}, yaml_path],
        #     # parameters=[{"robot_description": urdf_path}, config_path], #second last, shows error
        #     # parameters=[params], #works best 
        #     # parameters=[{"robot_description": urdf_path}],
        #     parameters=[{
        #                 'robot_description': ParameterValue(robot_desc, value_type=str)
        #                 }],
        #     # parameters=[{'robot_description': Command(['cat ', urdf_path])}],  #works best when using the urdf as urdf requires the entire descrtrription and not the path
        #     arguments=['--ros-args', '--log-level', 'info'] #added logging to inspect errors
        # ),

#-----------------------------------------------------------------------------------------------------------------
        # Joint State Publisher Node
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),

        Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{'robot_description': '<your_robot_description>'}],
                remappings=[
                    ('/robot_description', '/robot_namespace/robot_description'),
                ],
                output='screen',
            ),

        # Node(
        #     package='joint_state_publisher_gui',  
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher_gui',
        #     output='screen'
        # ),

        Node(
                package='controller_manager',
                executable='ros2_control_node',
                namespace='/robot_namespace',
                name='controller_manager',
                parameters=[
                    # {'robot_description': robot_desc},

                    robot_description,
                    config_path  # Load your YAML configuration
                ],
                 arguments=['--ros-args', '--log-level', 'debug'],
                output='screen'
            ),


            # adding a delay to ensure the robot is spawned before the controllers are loaded

            TimerAction(
                        period=5.0,  # Delay for 5 seconds
                        actions=[

                            Node(
                                package='controller_manager',
                                executable='spawner',
                                namespace='/robot_namespace' , 
                                arguments=['joint_state_broadcaster'],
                                name='joint_state_broadcaster_spawner',
                                output='screen',
                            ),

                            # Diff Drive Controller manager
                            # Node(
                            #     package='controller_manager',
                            #     executable='spawner',
                            #     namespace='/robot_namespace',
                            #     arguments=['diff_drive_controller'],
                            #     name='diff_drive_controller_spawner',
                            #     output='screen',
                            #     parameters=[PathJoinSubstitution([
                            #     FindPackageShare('test_diff_robot'),  # Replace with your package name
                            #     'config',
                            #     'diff_drive_controller.yaml'
                            #     ])],
                            # ),

                            Node(
                                package='controller_manager',
                                executable='spawner',
                                namespace='/robot_namespace',
                                arguments=['diff_drive_controller'],
                                name='diff_drive_controller_spawner',
                                output='screen',
                                # parameters=[PathJoinSubstitution([
                                # FindPackageShare('test_diff_robot'),  # Replace with your package name
                                # 'config',
                                # 'diff_drive_controller.yaml'
                                # ])],
                                parameters=[robot_desc,config_path],
                            )
                            ]
            ),          


        # Launch Gazebo with an empty world
        ExecuteProcess(
            # cmd=['ros2', 'launch', 'gazebo_ros', 'gazebo.launch.py', '--world', 'empty.sdf'],
            cmd=['gz', 'sim', 'empty.sdf'], #successfully launches the empty world
            output='screen'
        ),

        # Wait for a few seconds to ensure Gazebo is up and running befre adding the URDF model
        TimerAction(
            period=5.0,  # Waiting for 5 seconds
            actions=[
                ExecuteProcess(
                    cmd=[
                        'gz', 'service', '-s', '/world/empty/create',
                        '--reqtype', 'gz.msgs.EntityFactory',
                        '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '1000',
                        '--req', f'sdf_filename: "{urdf_path}", name: "urdf_model"'
                    ],
                    output='screen'
                )
            ]
        ), #spawining successfully done

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=["-d", rviz_config_file],
            output='screen',
            # arguments=['-d', os.path.join(pkg_share_description, 'config', 'rviz_config.rviz')]  # Adjust path to your RViz config file if you have one
        ),


        
    ])
