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

    # MID 360 3D lidar
    node_pcd2 = Node(
        package    = 'livox_to_pointcloud2',
        executable = 'livox_to_pointcloud2_node',
        name       = 'livox_to_pointcloud2',
        remappings = [('/livox_pointcloud', '/livox/lidar'), ('/converted_pointcloud2', '/livox/pcd2')]
    )
    ld.add_action(node_pcd2)
    
    # fast LIO mapping
    other_launch_file = os.path.join( get_package_share_directory('fast_lio'), 'launch', 'mapping.launch.py' )
    nodes_fastlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(other_launch_file),
        launch_arguments={'some_arg': 'value'}.items()
    )
    ld.add_action(nodes_fastlio)    

    return ld

