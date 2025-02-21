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
    
    # rosbag
    #bag_file = '/home/ubuntu/Downloads/outdoor_hard_01a'
    #bag_file = '/home/ubuntu/Downloads/outdoor_hard_01b'
    #bag_file = '/home/ubuntu/Downloads/outdoor_hard_02a'
    #bag_file = '/home/ubuntu/Downloads/outdoor_hard_02b'
    bag_file = '/home/ubuntu/Downloads/outdoor_hard_02a'    
    node_rosbag = ExecuteProcess(
        cmd    = ['ros2','bag','play',bag_file],
        output = 'screen',
    )
    ld.add_action(node_rosbag)    

    # Map Publisher
    node_map = Node(
        package    = 'pcl_ros',
        executable = 'pcd_to_pointcloud',
        name       = 'map_publisher',
        output     = 'screen',
        parameters = [{'file_name': '/home/ubuntu/Downloads/outdoor.pcd', 'tf_frame': 'map'}],
        remappings = [('cloud_pcd','global_map')]
    )
    ld.add_action(node_map)

    rviz_config_file = PathJoinSubstitution([ FindPackageShare('ros2_nicorover2'), 'config', 'localization.rviz' ])
    node_rviz = Node(
        package    = 'rviz2',
        executable = 'rviz2',
        name       = 'rviz2',
        namespace  = 'rviz2',
        arguments  = ['-d', rviz_config_file]
    )
    ld.add_action(node_rviz)

    return ld

