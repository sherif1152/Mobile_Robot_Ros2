

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


MAP_NAME='map' #change to the name of your own map here

def generate_launch_description():

    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('robot_description_pkg'), 'launch', 'gazebo.launch.py']
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare('robot_navigation_pkg'), 'rviz', 'navigation.rviz']
    )

    default_map_path = PathJoinSubstitution(
        [FindPackageShare('robot_navigation_pkg'), 'maps', f'{MAP_NAME}.yaml']
    )

    nav2_config_path = PathJoinSubstitution(
        [FindPackageShare('robot_navigation_pkg'), 'config', 'nav.yaml']
    )
    

    # nav2_sim_config_path = PathJoinSubstitution(
    #     [FindPackageShare('robot_navigation_pkg'), 'config', 'nav_sim.yaml']
    # )


    return LaunchDescription([
        DeclareLaunchArgument(
            name='sim', 
            default_value='false',
            description='Enable use_sime_time to true'
        ),

        # DeclareLaunchArgument(
        #     name='rviz', 
        #     default_value='false',
        #     description='Run rviz'
        # ),

       DeclareLaunchArgument(
            name='map', 
            default_value=default_map_path,
            description='Navigation map path'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            condition=UnlessCondition(LaunchConfiguration("sim")),
            launch_arguments={
                'map': LaunchConfiguration("map"),
                'use_sim_time': LaunchConfiguration("sim"),
                'params_file': nav2_config_path
            }.items()
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(nav2_launch_path),
        #     condition=IfCondition(LaunchConfiguration("sim")),
        #     launch_arguments={
        #         'map': LaunchConfiguration("map"),
        #         'use_sim_time': LaunchConfiguration("sim"),
        #         'params_file': nav2_sim_config_path
        #     }.items()
        # ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])