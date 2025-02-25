#!/usr/bin/env python3
# coding: utf-8

import os
from launch                            import LaunchDescription
from launch.actions                    import IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions                import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions              import PathJoinSubstitution
from launch_ros.substitutions          import FindPackageShare
from ament_index_python.packages       import get_package_share_directory




def generate_launch_description():
    ld = LaunchDescription()
    
    use_sim_time = False
    
    # global map publisher
    node_cart = Node(
        package    = 'cartographer_ros',
        executable = 'cartographer_node',
        name       = 'cartographer_node',
        output     = 'screen',
        parameters = [{
            'use_sim_time': use_sim_time,
            'scan_topic': '/scan',
        }],
        arguments=[
            '--configuration_directory', '/home/sajisaka/ros2_ws/src/nicorover2/ros2_nicorover2/config',
            '--configuration_basename', 'cartgrapher_config.lua'
        ],        
    )
    ld.add_action(node_cart)

    # global localizer    
    node_grid = Node(
        package    = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        name       = 'cartographer_occupancy_grid_node',
        output     = 'screen',
        parameters = [{
            'use_sim_time' : use_sim_time
        }]
    )
    ld.add_action(node_grid)

    
    return ld

