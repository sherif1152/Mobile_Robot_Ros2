#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    model_path = os.path.join(
        get_package_share_directory('robot_description_pkg'),
        'models',
        'model.sdf'
    )
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
    urdf_path = os.path.join(
        get_package_share_directory('robot_description_pkg'),
        'urdf',
        'robot.urdf'    
    )

    spawn_robot = LaunchConfiguration('spawn_robot', default='true')

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

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

    myrobot = 'my_robot'
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify x position of the robot'
    )

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify y position of the robot'
    )

    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', myrobot,
            '-file', model_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
        output='screen',
    )
    
    print("STARTING ALL NODES")

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    ld = LaunchDescription()

    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)

    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(rviz_node)
    ld.add_action(start_gazebo_ros_spawner_cmd)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    return ld
