#!/usr/bin/env python3
# coding: utf-8

import time
import open3d as o3d
import numpy  as np
from scipy.spatial.transform import Rotation as R

import rclpy
import tf2_ros
from tf2_ros           import TransformBroadcaster
from rclpy.node        import Node
from sensor_msgs.msg   import PointCloud2, Imu
from nav_msgs.msg      import Odometry
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, Pose, Point, Quaternion
import sensor_msgs_py.point_cloud2 as pc2

from .global_localizer import global_localizer




class ros2_global_localizer(Node):

    def __init__(self):
        # initialize node
        super().__init__('global_localizer')

        # declare parameter
        self.declare_parameter( 'MAP_VOXEL_SIZE',    0.4     )
        self.declare_parameter( 'SCAN_VOXEL_SIZE',   0.1     )
        self.declare_parameter( 'FREQ_LOCALIZATION', 0.5     )
        self.declare_parameter( 'LOCALIZATION_TH',   0.8     )        
        self.declare_parameter( 'FOV',               6.28319 )
        self.declare_parameter( 'FOV_FAR',           300     )
        self.declare_parameter( 'MAP_FRAME_ID',      'map'   )

        # read parameter
        self._MAP_VOXEL_SIZE    = float(self.get_parameter('MAP_VOXEL_SIZE').value)
        self._SCAN_VOXEL_SIZE   = float(self.get_parameter('SCAN_VOXEL_SIZE').value)
        self._FREQ_LOCALIZATION = float(self.get_parameter('FREQ_LOCALIZATION').value)
        self._LOCALIZATION_TH   = float(self.get_parameter('LOCALIZATION_TH').value)
        self._FOV               = float(self.get_parameter('FOV').value)
        self._FOV_FAR           = float(self.get_parameter('FOV_FAR').value)
        self._MAP_FRAME_ID      = self.get_parameter('MAP_FRAME_ID').value        

        # initialize topic
        # subscriber
        self._sub_pcd2 = self.create_subscription( PointCloud2,               'cloud_registered', self._callback_pcd2, 1 ) # registrated pointcloud
        self._sub_odom = self.create_subscription( Odometry,                  'Odometry',         self._callback_odom, 1 ) # Odometry from FAST-LIO
        # 
        self._sub_pose = self.create_subscription( PoseWithCovarianceStamped, 'initialpose',      self._callback_pose, 1 ) # initial pose
        self._sub_map  = self.create_subscription( PointCloud2,               'map',              self._callback_map,  1 ) # global map pointcloud
        
        # publisher
        self._pub_odom     = self.create_publisher( Odometry,    'map_to_odom',     1 )
        
        # saved topic        
        self._globalmap_pc2 = None  # global map pointcloud (PointCloud2)
        self._scanmap_pc2   = None  # scan pointcloud (PointCloud2)
        self._current_odom  = None  # odometry(Odometry)
        self._initialpose   = None  # initial pose(PoseWithCovarianceStamped)
        
        self._mat_map_to_odom       = np.eye(4)
        
        # global localizer
        self._gl = global_localizer()
        
        _init_pos = Pose()
        _init_pos.position.x    = -104.346385
        _init_pos.position.y    = -10.3209830
        _init_pos.position.z    = -11.6631100
        _init_pos.orientation.x = -0.03533900
        _init_pos.orientation.y =  0.04606000
        _init_pos.orientation.z =  0.98759600
        _init_pos.orientation.w =  0.14588700
        
        self._mat_map_to_odom   = self._posemsg_to_mat44(_init_pos)
        
        # start timer
        self.timer = self.create_timer(1.0/self._FREQ_LOCALIZATION, self._callback_timer)


    # callback
    # scan pointcloud2
    def _callback_pcd2( self, msg ):
        self._scanmap_pc2   = msg
        
        return

        
    # global map pointcloud2
    def _callback_map( self, msg ):
        self._globalmap_pc2 = msg
        return


    # odometry (from FAST_LIO)
    def _callback_odom( self, msg ):
        self._current_odom = msg
        return


    # iniital pose
    def _callback_pose( self, msg ):
        self._initialpose = msg        
        return
        
    
    # timer
    def _callback_timer(self):
        # check data
        if( self._globalmap_pc2 == None ):
            print( 'global map is not received' )
            return
            
        if( self._scanmap_pc2 == None ):
            print( 'scan map is not received' )
            return
            
        if( self._current_odom == None ):
            print( 'odometry is not received' )
            return
                
        # convert pc2 to open3D
        _gm_o3d   = self._pc2msg_to_o3dpc(self._globalmap_pc2)
        _scan_o3d = self._pc2msg_to_o3dpc(self._scanmap_pc2)
        
        # get odometry
        # convert ocometry to 4x4 matrix
        _trans = self._posemsg_to_mat44(self._current_odom.pose.pose)
        _trans = self._mat_map_to_odom
        
        # IPC matching
        # rough matching
        ( _trans, _fitness ) = self._gl.registration_at_scale(_scan_o3d, _gm_o3d, _trans, 5.0)
        # matching
        ( _trans, _fitness ) = self._gl.registration_at_scale(_scan_o3d, _gm_o3d, _trans, 1.0)
        
        if( _fitness > self._LOCALIZATION_TH ):
            # convert matched point to Pose msessage
            self._mat_map_to_odom = _trans
        
            _odom = Odometry()
            _odom.pose.pose       = self._mat44_to_posemsg(self._mat_map_to_odom)
            _odom.header.stamp    = self.get_clock().now().to_msg()
            _odom.header.frame_id = self._MAP_FRAME_ID
            self._pub_odom.publish(_odom)
            
        return
        
        
    # convert PointCloud2 to nparray
    def _pc2msg_to_o3dpc(self, msg):
        _pts = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        _npt = np.array([(_p[0], _p[1], _p[2]) for _p in list(_pts)], dtype=np.float32)
        
        _ret = o3d.geometry.PointCloud()
        _ret.points = o3d.utility.Vector3dVector(_npt)
        
        return _ret
        
    
    # convert Pose Message to Matrix 4x4
    def _posemsg_to_mat44(self, msg):
        _p = [msg.position.x,    msg.position.y,    msg.position.z]
        _q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

        _r = R.from_quat(_q)
        _T = np.eye(4)
        _T[:3, :3] = _r.as_matrix()
        _T[:3, 3]  = _p
        
        return _T


    # convert Matrix 4x4 to Pose Message
    def _mat44_to_posemsg(self, mat):
        _p = mat[:3, 3]
        _r = R.from_matrix(mat[:3, :3])
        _q = _r.as_quat()
        
        _pose = Pose()
        _pose.position.x    = _p[0]
        _pose.position.y    = _p[1]
        _pose.position.z    = _p[2]        
        _pose.orientation.x = _q[0]
        _pose.orientation.y = _q[1]
        _pose.orientation.z = _q[2]
        _pose.orientation.w = _q[3]
        
        return _pose




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

