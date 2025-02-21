#!/usr/bin/env python3
# coding: utf-8

import rclpy
from rclpy.node        import Node
from sensor_msgs.msg   import PointCloud2, Imu
from nav_msgs.msg      import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import sensor_msgs_py.point_cloud2 as pc2

import time
import open3d as o3d
import numpy  as np

from .global_localizer import global_localizer


class ros2_global_localizer(Node):

    def __init__(self):
        # initialize node
        super().__init__('global_localizer')

        # declare parameter
        self.declare_parameter( 'MAP_VOXEL_SIZE',    0.4  )
        self.declare_parameter( 'SCAN_VOXEL_SIZE',   0.1  )
        self.declare_parameter( 'FREQ_LOCALIZATION', 0.5  )
        self.declare_parameter( 'LOCALIZATION_TH',   0.95 )        
        self.declare_parameter( 'FOV',               1.6  )
        self.declare_parameter( 'FOV_FAR',           150  )

        # read parameter
        self._MAP_VOXEL_SIZE    = float(self.get_parameter('MAP_VOXEL_SIZE').value)
        self._SCAN_VOXEL_SIZE   = float(self.get_parameter('SCAN_VOXEL_SIZE').value)
        self._FREQ_LOCALIZATION = float(self.get_parameter('FREQ_LOCALIZATION').value)
        self._LOCALIZATION_TH   = float(self.get_parameter('LOCALIZATION_TH').value)
        self._FOV               = float(self.get_parameter('FOV').value)
        self._FOV_FAR           = float(self.get_parameter('FOV_FAR').value)

        # initialize topic
        self._sub_pcd2 = self.create_subscription( PointCloud2,               'pcd_in',      self._callback_pcd2, 10 )
        self._sub_map  = self.create_subscription( PointCloud2,               'global_map',  self._callback_map,  10 )
        self._sub_odom = self.create_subscription( Odometry,                  'odom',        self._callback_odom, 10 )
        self._sub_pose = self.create_subscription( PoseWithCovarianceStamped, 'initialpose', self._callback_pose, 10 )
        
        # publisher
        self._pub_odom = self.create_publisher( Odometry, 'odom_estimated', 10 )

        self._global_map_received       = False
        self._initial_position_received = False
        
        # global localizer
        self._gl = global_localizer()
        
        # start timer
        self.timer = self.create_timer(1.0/self._FREQ_LOCALIZATION, self._callback_timer)


    # callback
    # scan pointcloud2
    def _callback_pcd2( self, msg ):
        return

        
    # map pointcloud2
    def _callback_map( self, msg ):
    
        # convert PointCloud2 to nparray
        points_np = self._msg_to_array(msg)
        
        # convert nparray to Open3D PointCloud
        gm        = o3d.geometry.PointCloud()
        gm.points = o3d.utility.Vector3dVector(points_np)
        
        # set global map to localizer
        self._gl.load_globalmap(gm)

        self._global_map_received = True        
        print('*** GLOBAL MAP RECEIVED ***')
        return

        
    # odometry (e.g. wheel odometry)
    def _callback_odom( self, msg ):
        return


    # iniital pose
    def _callback_pose( self, msg ):
        self._initial_position_received = True
        print('*** INITIAL POSE RECEIVED ***')
        return
        
    
    # timer
    def _callback_timer(self):
        ret = Odometry()
        self._pub_odom.publish(ret)
        return
        
        
    # convert PointCloud2 to nparray
    def _msg_to_array(self, msg):
        points      = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points_list = list(points)
        points_np   = np.array([(p[0], p[1], p[2]) for p in points_list], dtype=np.float32)
        return points_np




# main function
def main(args=None):
    # initialize ROS2
    rclpy.init(args=args)

    # initialize node
    node = ros2_global_localizer()
    rclpy.spin(node)

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()




# main function
if(__name__ == '__main__'):
    main()
    
    
"""
FAST_LIO_LOCALIZATION
globalな自己意思推定(0.2〜0.5Hz)と、その間を補完する短距離の自己位置推定の組み合わせ。
グローバルな方がglobal_localizerで、ローカルな方がtansform_fusion担っている。
で、ローカルな方はLIO_SAMベース(要するに自己位置とIMUを組み合わせているって話かな？)

グローバルは
入力：
map (Pointcloud2)
スキャン(PointCloud2)
オドメトリ(Odometry)
初期位置(PoseWithCovariantStamped)
出力：
オドメトリ(Odometry)
担っているっぽい（あとはデバッグ用のポイントクラウド？）
"""
