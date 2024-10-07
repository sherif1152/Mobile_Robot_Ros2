#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    urdf_path = os.path.join(
        get_package_share_directory('robot_description_pkg'),
        'models', 'model.sdf'   
    )

    # Get RViz config file
    rviz_config = os.path.join(
        get_package_share_directory('robot_description_pkg'),
        'rviz',
        'view_robot.rviz'
    )

    world = os.path.join(
        get_package_share_directory('robot_description_pkg'),
        'world',
        'wall.world'
    )

    # Define LaunchConfiguration for spawning the robot
    spawn_robot = LaunchConfiguration('spawn_robot', default='true')

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Define parameters
    common_params = {
        'robot_description': robot_desc,
    }

    # Add nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[common_params]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    rviz_node = Node(
        condition=IfCondition(spawn_robot),
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', rviz_config]
    )

    myrobot = 'robot_name' 
    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify the x-coordinate for the robot spawn position'
    )

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify the y-coordinate for the robot spawn position'
    )

    # Launch Ignition Gazebo (gz sim) with the specified world file
    gz_sim_cmd = ExecuteProcess(
        cmd=['gz', 'sim', world],
        output='screen'
    )

    # Use Ignition Gazebo to spawn the robot into the simulation
    spawn_robot_cmd = ExecuteProcess(
        cmd=[
            'gz', 'sim', '-v', '4', world,
            '--spawn-file', urdf_path,
            '--name', myrobot,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
        output='screen'
    )

    # Create launch description
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)

    # Add the nodes and processes to launch description
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(rviz_node)
    ld.add_action(gz_sim_cmd)
    ld.add_action(spawn_robot_cmd)

    return ld
