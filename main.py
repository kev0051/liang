import geopandas as gpd
import open3d as o3d
import tools.lasTest as lasTest
import tools.voxelize as voxelization
import tools.data as data

print('bruh')

xmin = 673029.1
bound = 50
ymin = 3678019.6 
lasdata = 'initData.las'
count = 0

#voxelization.view_as_pointcloud(xmin, ymin, bound, lasdata)

# Reference for saving images: http://www.open3d.org/docs/release/tutorial/visualization/non_blocking_visualization.html
# http://www.open3d.org/docs/release/python_api/open3d.visualization.Visualizer.html
# https://github.com/isl-org/Open3D/issues/1095

# Change the range for number of images to save
for i in (n+7 for n in range (1)): # n+6 starts saving at train_0005, range(1) saves 1 image for that config
    v = lasTest.get_voxelization_heat(xmin, ymin, bound, 0.75, lasdata) 
    xmin = xmin + (bound/2) # set for half-window configuration for now
    #voxelization.visualize_voxel_grid(v) # comment this out if you don't want to pull up the image for yourself to view
    
    #comment this out if you don't want to save the image
    voxelization.capture_voxel_grid(v, i, 'training_data') # i is for filename, string is folder to save to (make sure folder exists in directory)
    
    count = count + 1

print("Saved", count, "images.")

# Same as what happens in loop but non heatmap version
#v = lasTest.get_voxelization(xmin, ymin, bound, 0.75, lasdata)
#voxelization.visualize_voxel_grid(v)

# Other approach stuff
#full = voxelization.fill_voxel_grid(v, 1, 25)
#voxelization.visualize_voxel_grid(full[0])
