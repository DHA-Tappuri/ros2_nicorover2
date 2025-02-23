#!/usr/bin/env python3
# coding: utf-8


import time, os
import open3d as o3d
import numpy  as np





class global_localizer(object):
    # constructor
    def __init__(self, map_voxel:float=0.4, scan_voxel:float=0.1, loc_th:float=0.95, fov:float=1.6, max_range:float=150.0):
        # read parameters
        self._map_voxel  = map_voxel
        self._scan_voxel = scan_voxel
        self._loc_th     = loc_th
        self._fov        = fov
        self._max_range  = max_range
        
        self._global_map = None


    # destructor
    def __del__(self):
        pass


    # load global map
    def load_globalmap(self, data):
        # downsize pointcloud
        self._global_map = self._voxel_down_sample( data, self._map_voxel )


    # down sample
    def _voxel_down_sample(self, pcd, voxel_size):
        try:
            ret = pcd.voxel_down_sample(voxel_size)
        except:
            ret = o3d.geometry.voxel_down_sample(pcd, voxel_size)
        return ret
        
    
    # ICP matching
    def registration_at_scale(self, pc_scan, pc_map, initial, scale):
        # downsample map and scan points
        _pc_scan = self._voxel_down_sample(pc_scan, self._scan_voxel * scale)
        _pc_map  = self._voxel_down_sample(pc_map,  self._map_voxel  * scale)
        # ICP matching
        _icp     = o3d.pipelines.registration.registration_icp( 
            _pc_scan, _pc_map, 1.0*scale, initial,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20)
        )
        
        # return result
        return ( _icp.transformation, _icp.fitness )




# main function
def main(args=None):
    pass




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
