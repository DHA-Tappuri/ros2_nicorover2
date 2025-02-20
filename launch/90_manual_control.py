#!/usr/bin/env python3
# coding: utf-8

import os
from launch                            import LaunchDescription
from launch.actions                    import IncludeLaunchDescription
from launch_ros.actions                import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions              import PathJoinSubstitution
from launch_ros.substitutions          import FindPackageShare
from ament_index_python.packages       import get_package_share_directory


def generate_launch_description():

    # delta2g 2D lidar
    ld = LaunchDescription()

    other_launch_file = os.path.join( get_package_share_directory('ros2_delta_lidar'), 'launch', '00_delta2g.launch.py' )
    print('====================================================================')
    print(other_launch_file)
    print('====================================================================')        
    """
    nodes_delta2g = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(other_launch_file),
        launch_arguments={'some_arg': 'value'}.items()
    )
    """
    nodes_delta2g = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(other_launch_file)
    )
    ld.add_action(nodes_delta2g)

    """
    # MID 360 3D lidar    
    other_launch_file = os.path.join( get_package_share_directory('ros2_delta_lidar'), 'launch', '00_delta2g.launch.py' )
    nodes_mid360 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(other_launch_file),
        launch_arguments={'some_arg': 'value'}.items()
    )
    ld.add_action(nodes_mid360)
    """

    """
    # DDSM controller
    other_launch_file = os.path.join( get_package_share_directory('ros2_ddsm_robot'), 'launch', '90_manual_control.launch.py' )
    nodes_ddsm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(other_launch_file),
        launch_arguments={'some_arg': 'value'}.items()
    )
    ld.add_action(nodes_ddsm)    
    """

    # rviz    
    rviz_config_file = PathJoinSubstitution([ FindPackageShare('ros2_nicorover2'), 'config', 'nicorover2.rviz' ])
    node_rviz = Node(
        package    = 'rviz2',
        executable = 'rviz2',
        name       = 'rviz2',
        namespace  = 'rviz2',
        arguments  = ['-d', rviz_config_file],
        output     = 'screen',
    )
    ld.add_action(node_rviz)

    return ld
