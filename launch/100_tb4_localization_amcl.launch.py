#!/usr/bin/env python3
# coding: utf-8

"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages       import get_package_share_directory
"""

import os
from launch                            import LaunchDescription
from launch.actions                    import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.substitutions              import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions                import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions              import PathJoinSubstitution
from launch_ros.substitutions          import FindPackageShare
from ament_index_python.packages       import get_package_share_directory



ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time', 
        default_value = 'true',
        choices       = ['true', 'false'],
        description   = 'Use sim time'
    ),
    DeclareLaunchArgument(
        'namespace', 
        default_value = '',
        description   = 'Robot namespace'
    )
]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    
    other_launch_file = os.path.join( get_package_share_directory('nav2_bringup'), 'launch', 'localization_launch.py' )
    nodes_local = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(other_launch_file),
        launch_arguments={
            'use_sim_time' : LaunchConfiguration('use_sim_time'),
            'namespace'    : '',
            'map'          : '/home/sajisaka/ros2_ws/src/nicorover2/ros2_nicorover2/config/map/warehouse.yaml',
            'params_file'  : '/home/sajisaka/ros2_ws/src/nicorover2/ros2_nicorover2/config/yaml/nav2_localization.yaml'
        }.items()
    )
    ld.add_action(nodes_local)
    
    return ld

