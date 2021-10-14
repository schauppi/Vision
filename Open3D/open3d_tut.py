import numpy as np
import open3d as o3d
import copy


source = o3d.io.read_point_cloud("/Users/davidschaupp/Documents/GitHub/Vision/Open3D/test_data/ICP/cloud_bin_0.pcd")
target = o3d.io.read_point_cloud("/Users/davidschaupp/Documents/GitHub/Vision/Open3D/test_data/ICP/cloud_bin_1.pcd")

def draw_registration_result(source, target, transformation):
    pass

source_temp = copy.deepcopy(source)
target_temp = copy.deepcopy(target)

source_temp.paint_uniform_color([1, 0.706, 0])
target_temp.paint_uniform_color([0, 0.651, 0.929])
#o3d.visualization.draw_geometries([source_temp, target_temp])

reg_p2p = o3d.pipelines.registration.registration_icp(source, target, 0.02)

o3d.visualization.draw_geometries([reg_p2p])
