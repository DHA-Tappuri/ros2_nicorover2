#!/usr/bin/env python3
# coding: utf-8

import time
import open3d as o3d
import numpy  as np
from scipy.spatial.transform import Rotation as R

import rclpy
import tf2_ros
from rclpy.node        import Node
from nav_msgs.msg      import Odometry
from geometry_msgs.msg import Pose, TransformStamped





class ros2_transform_fusion(Node):

    def __init__(self):
        # initialize node
        super().__init__('transform_fusion')
        
        # declare parameter
        self.declare_parameter( 'FREQ_PUB_LOCALIZATION', 50            )
        self.declare_parameter( 'MAP_FRAME_ID',          'map'         )
        self.declare_parameter( 'BASE_FRAME_ID',         'body'        )
        self.declare_parameter( 'LIDAR_FRAME_ID',        'camera_init' )

        # read parameter
        self._FREQ_PUB_LOCALIZATION = float(self.get_parameter('FREQ_PUB_LOCALIZATION').value)
        self._MAP_FRAME_ID          = self.get_parameter('MAP_FRAME_ID').value
        self._BASE_FRAME_ID         = self.get_parameter('BASE_FRAME_ID').value
        self._LIDAR_FRAME_ID        = self.get_parameter('LIDAR_FRAME_ID').value

        # initialize topic
        # subscriber
        self._sub_global = self.create_subscription( Odometry, 'map_to_odom', self._callback_globalpos, 1 )  # global position
        self._sub_odom   = self.create_subscription( Odometry, 'Odometry',    self._callback_odom,      1 )  # Odometry from FAST-LIO
        # publisher        
        self._pub_loc   = self.create_publisher( Odometry, 'localization', 10 )
        
        self._odom_to_baselink = None # (Odometry) odometry to baselink (FAST-LIO odometry)
        self._map_to_odom      = None # (Odometry) map to position (global position)
        self._tf_caster        = tf2_ros.TransformBroadcaster(self)

        # global position matrix
        self._mat_map_to_odom       = np.eye(4)
        self._mat_odom_to_base_link = np.eye(4)
        
        # timer function
        self.timer = self.create_timer(1.0/self._FREQ_PUB_LOCALIZATION, self._callback_timer)

        return


    # callback
    # global position
    def _callback_globalpos( self, msg ):
        self._map_to_odom = msg    
        return


    # odometry
    def _callback_odom( self, msg ):
        self._odom_to_baselink = msg
        return


    # timer callback
    def _callback_timer(self):
        # get matrix
        if( self._map_to_odom is not None ):
            self._mat_map_to_odom = self._posemsg_to_mat44(self._map_to_odom.pose.pose)
        
        # send transform from map to odom (global position)
        _pose = self._mat44_to_posemsg(self._mat_map_to_odom)
        _t = TransformStamped()
        _t.header.stamp            = self.get_clock().now().to_msg()
        _t.header.frame_id         = self._MAP_FRAME_ID
        _t.child_frame_id          = self._LIDAR_FRAME_ID
        _t.transform.translation.x = _pose.position.x
        _t.transform.translation.y = _pose.position.y
        _t.transform.translation.z = _pose.position.z
        _t.transform.rotation      = _pose.orientation
        self._tf_caster.sendTransform(_t)


        if( self._odom_to_baselink is not None ):
            _loc = Odometry()
            self._mat_odom_to_base_link = self._posemsg_to_mat44(self._odom_to_baselink.pose.pose)
            
            _mat_map_to_base_link = np.matmul(self._mat_map_to_odom, self._mat_odom_to_base_link)
            
            _loc.pose.pose       = self._mat44_to_posemsg(_mat_map_to_base_link)
            _loc.twist           = self._odom_to_baselink.twist

            _loc.header.stamp    = self._odom_to_baselink.header.stamp
            _loc.header.frame_id = self._MAP_FRAME_ID
            _loc.child_frame_id  = self._BASE_FRAME_ID
            
            self._pub_loc.publish( _loc )


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
    node = ros2_transform_fusion()
    rclpy.spin(node)

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()




# main function
if(__name__ == '__main__'):
    main()

