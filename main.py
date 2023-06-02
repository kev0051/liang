import geopandas as gpd
import open3d as o3d
import tools.lasTest as lasTest
import tools.voxelize as voxelization
import tools.data as data
#import AI.ai1 as ai1

print('bruh')

xmin = 671918.4
ymin = 3678564.1
bound = 25
lasdata = 'initData.las'

#voxelization.view_as_pointcloud(xmin, ymin, bound, lasdata)
v = lasTest.get_voxelization_heat(xmin, ymin, bound, 0.35, lasdata)

voxelization.visualize_voxel_grid(v)
#voxelization.rotate(v, 5, 0,)

full = voxelization.fill_voxel_grid(v, 1, 25)

#voxelization.visualize_voxel_grid(full[0])