import geopandas as gpd
import open3d as o3d
import tools.lasTest as lasTest
import tools.voxelize as voxelization
import tools.data as data

print('bruh')

xmin = 671843.7 
bound = 50
ymin = 3677206.3 
lasdata = 'initData.las'
count = 0

#voxelization.view_as_pointcloud(xmin, ymin, bound, lasdata)

# -- IMAGE CAPTURE STARTS HERE --

# Reference for saving images: http://www.open3d.org/docs/release/tutorial/visualization/non_blocking_visualization.html
# http://www.open3d.org/docs/release/python_api/open3d.visualization.Visualizer.html
# https://github.com/isl-org/Open3D/issues/1095

# Change the range for number of images to save
#for i in (n+19 for n in range (20)): # n+6 starts saving at x_0005, range(1) saves 1 image for that config
#    v = lasTest.get_voxelization_heat(xmin, ymin, bound, 0.75, lasdata) 
#    xmin = xmin + (bound/2) # set for half-window configuration for now
    #voxelization.visualize_voxel_grid(v) # comment this out if you don't want to pull up the image for yourself to view
    
    #comment this out if you don't want to save the image
#    voxelization.capture_voxel_grid(v, i, 'images/train') # i is for filename, string is folder to save to (make sure folder exists in directory)
    
#    count = count + 1

#print("Saved", count, "images.")

# -- IMAGE CAPTURE ENDS HERE --

# Same as what happens in loop but non heatmap version
#v = lasTest.get_voxelization(xmin, ymin, bound, 0.75, lasdata)
#voxelization.visualize_voxel_grid(v)

# Other approach stuff
#full = voxelization.fill_voxel_grid(v, 1, 25)
#voxelization.visualize_voxel_grid(full[0])

# -- ISOLATION ALGORITHM STARTS HERE --

v = voxelization.get_tree(672757.4, 3678095.1, 672779.9, 3678120.1, lasdata)
voxelization.visualize_voxel_grid(v)
