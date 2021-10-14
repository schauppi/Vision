import open3d as o3d

pcd_000 = o3d.io.read_point_cloud("/Users/davidschaupp/Documents/GitHub/Vision/Open3D/Stanford_Bunny_ICP/bunny/data/bun000.ply")
pcd_090 = o3d.io.read_point_cloud("/Users/davidschaupp/Documents/GitHub/Vision/Open3D/Stanford_Bunny_ICP/bunny/data/bun090.ply")
o3d.visualization.draw_geometries([pcd_000])
o3d.visualization.draw_geometries([pcd_090])
