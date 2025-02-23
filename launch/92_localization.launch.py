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
    
    use_sim_time = True

    # fast LIO mapping
    other_launch_file = os.path.join( get_package_share_directory('fast_lio'), 'launch', 'mapping.launch.py' )
    nodes_fastlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(other_launch_file),
        launch_arguments={'some_arg': 'value'}.items()
    )
    ld.add_action(nodes_fastlio)
    
    # rosbag
    bag_file = '/home/sajisaka/ros2_ws/src/nicorover2/outdoor_hard_02a'    
    node_rosbag = ExecuteProcess(
        cmd = [
            'ros2', 'bag', 'play', bag_file, '--clock',
        ],
        output = 'screen',
    )
    ld.add_action(node_rosbag)

    # global map publisher
    node_map = Node(
        package    = 'pcl_ros',
        executable = 'pcd_to_pointcloud',
        name       = 'map_publisher',
        output     = 'screen',
        parameters = [{
            'file_name'    : '/home/sajisaka/ros2_ws/src/nicorover2/ros2_nicorover2/testdata/outdoor.pcd', 
            'tf_frame'     : 'map',
            'use_sim_time' : use_sim_time
        }],
        remappings = [('cloud_pcd','map')]
    )
    ld.add_action(node_map)

    # global localizer    
    node_global = Node(
        package    = 'ros2_nicorover2',
        executable = 'global_localizer',
        name       = 'global_localizer',
        output     = 'screen',
        parameters = [{
            'use_sim_time' : use_sim_time
        }]
    )
    ld.add_action(node_global)
    
    # fusion transform    
    node_fusion = Node(
        package    = 'ros2_nicorover2',
        executable = 'transform_fusion',
        name       = 'transform_fusion',
        output     = 'screen',
        parameters = [{
            'use_sim_time' : use_sim_time
        }]
    )
    ld.add_action(node_fusion)

    # rviz
    """
    rviz_config_file = PathJoinSubstitution([ FindPackageShare('ros2_nicorover2'), 'config', 'localization.rviz' ])
    node_rviz = Node(
        package    = 'rviz2',
        executable = 'rviz2',
        name       = 'rviz2',
        namespace  = 'rviz2',
        output     = 'screen',
        parameters = [{
            'use_sim_time' : use_sim_time
        }],
        arguments  = ['-d', rviz_config_file]
    )
    ld.add_action(node_rviz)
    """

    return ld

