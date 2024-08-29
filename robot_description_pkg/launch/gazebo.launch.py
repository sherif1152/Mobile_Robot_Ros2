#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():

    # Get dir names
    pkg_share = get_package_share_directory('robot_description_pkg')

    # Get rviz config
    rviz_config = PathJoinSubstitution([
        FindPackageShare('robot_description_pkg'), 'rviz/view_robot.rviz'])
        
    world_path = PathJoinSubstitution(
        [FindPackageShare("robot_description_pkg"), "world/wall.world"])

    # Add launch description variables
    namespace = LaunchConfiguration('namespace')
    spawn_robot = LaunchConfiguration('spawn_robot')

    # Add launch arguments
    namespace_arg = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Node namespace'
    )

    spawn_robot_arg = DeclareLaunchArgument(
        name='spawn_robot',
        default_value='true',
        description='Flag to spawn the robot or not'
    )

    # Compute robot_description string
    xacro_file = os.path.join(pkg_share, 'urdf/robot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    # Define parameters
    common_params = {
        'robot_description': robot_description,
    }

    # Add nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        namespace=namespace,
        parameters=[common_params]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        namespace=namespace
    )

    rviz_node = Node(
        condition=IfCondition(spawn_robot),
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        namespace=namespace,
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        namespace_arg,
        spawn_robot_arg,
        DeclareLaunchArgument(
            name='world',
            default_value=world_path,
            description='Gazebo world'
        ),
        
        
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so', LaunchConfiguration('world')],
            output='screen'
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'linorobot2',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.01',
            ]
        ),
        joint_state_publisher,
        robot_state_publisher,
        rviz_node
    ])
