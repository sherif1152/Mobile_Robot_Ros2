#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    # Declare variables for use in the launch file
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Get directories for Gazebo and robot description packages
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_robot_description = get_package_share_directory('robot_description_pkg')

    # Set paths for the model, URDF, RVIZ, and world files
    model_path = os.path.join(pkg_robot_description, 'models', 'model.sdf')
    urdf_path = os.path.join(pkg_robot_description, 'urdf', 'robot.urdf')
    rviz_config_path = os.path.join(pkg_robot_description, 'rviz', 'view_robot.rviz')
    world_path = os.path.join(pkg_robot_description, 'world', 'word.world')

    # Declare launch arguments for robot pose
    declare_x_position_cmd = DeclareLaunchArgument('x_pose', default_value='0.0', description='x position of the robot')
    declare_y_position_cmd = DeclareLaunchArgument('y_pose', default_value='0.0', description='y position of the robot')

    # Read the URDF file and set the robot description parameter
    with open(urdf_path, 'r') as urdf_file:
        robot_desc = urdf_file.read()

    # Define the robot_state_publisher node to publish the robot's state from the URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }],
    )

    # Define the joint_state_publisher node
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # Define the RViz2 node
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('spawn_robot', default='true')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    # Define the Gazebo server and client nodes
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Define the robot spawning node with a delay to ensure Gazebo is ready
    spawn_entity_cmd = TimerAction(
        period=5.0,  # Wait for Gazebo to start up
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'my_robot',
                    '-file', model_path,
                    '-x', LaunchConfiguration('x_pose'),
                    '-y', LaunchConfiguration('y_pose'),
                    '-z', '0.01'
                ],
                output='screen'
            )
        ]
    )

    # Create and return the launch description
    ld = LaunchDescription()

    # Add the position arguments
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)

    # Add the nodes to launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(rviz_node)
    ld.add_action(spawn_entity_cmd)

    return ld