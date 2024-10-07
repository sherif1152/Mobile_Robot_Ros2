#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():

    # Path to SDF file (since you're using Ignition Gazebo)
    sdf_path = os.path.join(
        get_package_share_directory('robot_description_pkg'),
        'models', 'model.sdf' 
    )

    # Path to RViz config file
    rviz_config = os.path.join(
        get_package_share_directory('robot_description_pkg'),
        'rviz',
        'view_robot.rviz'
    )

    # Launch configuration for spawning the robot
    spawn_robot = LaunchConfiguration('spawn_robot', default='true')

    # Define parameters (SDF is not used for robot_description, so we skip that)
    common_params = {}

    # Node: Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[common_params]
    )

    # Node: Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # Node: RViz
    rviz_node = Node(
        condition=IfCondition(spawn_robot),
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', rviz_config]
    )

    # Set robot name and pose launch configurations
    myrobot = 'robot_name'
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # Declare launch arguments for robot pose
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify the x-coordinate for the robot spawn position'
    )

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify the y-coordinate for the robot spawn position'
    )

    # Command to launch Ignition Gazebo (gz sim)
    gz_sim_cmd = ExecuteProcess(
        cmd=['gz', 'sim'],
        output='screen'
    )

    # Command to spawn the robot in Ignition Gazebo
    spawn_robot_cmd = ExecuteProcess(
        cmd=[
            'gz', 'sim', '--spawn-file', sdf_path,
            '--name', myrobot,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
        output='screen'
    )

    # Create launch description
    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)

    # Add nodes and commands to the launch description
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(rviz_node)
    ld.add_action(gz_sim_cmd)
    ld.add_action(spawn_robot_cmd)

    return ld
