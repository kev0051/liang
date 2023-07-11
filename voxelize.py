import open3d as o3d
import laspy
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from random import *

def view_as_pointcloud(xmin : float, ymin : float, bound : int, las_file : str):
    xmax = xmin + bound
    ymax = ymin + bound
    las = laspy.read(las_file)

    point_data = np.stack([las.x, las.y, las.z, las.classification], axis = 0).transpose((1,0))

    x_data = []
    y_data = []
    z_data = []

    for i in point_data:
        if i[0] > xmin and i[0] <  xmax and i[1] > ymin and i[1] < ymax:
            x_data.append(i[0])
            y_data.append(i[1])
            z_data.append(i[2])

    points = []

    zmin = min(z_data)
    for i in range(len(x_data)):
        points.append([x_data[i] - xmin, y_data[i] - ymin, z_data[i] - zmin])
    
    fig = plt.figure(figsize=(5, 5))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(x_data, y_data, z_data)
    ax.set_axis_off()
    plt.show()

def voxelize(points : list, colors : list, voxel_size : int, bound : int): # num voxels gained from here
    # Initialize a point cloud object
    pcd = o3d.geometry.PointCloud()

    # Add the points, colors and normals as Vectors
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    #Create a voxel grid from the point cloud with a voxel_size of voxel_size and with a minimum bound of 0 and maximum bound of bound
    mins = np.array([0, 0, 0])      
    maxs = np.array([bound, bound, bound])
    voxel_grid=o3d.geometry.VoxelGrid.create_from_point_cloud_within_bounds(pcd, voxel_size, mins, maxs)

    return voxel_grid


def fill_voxel_grid(v : o3d.geometry.VoxelGrid, voxel_size : int, bound : int):
    #make new pointcloud and then a new voxelGrid
    fullPoints = []
    fullColors = []
    xyz = []
    for x in range(int(bound/voxel_size)):
        for y in range(int(bound/voxel_size)):
            for z in range(int(bound/voxel_size)):
                xyz.append([x*voxel_size, y*voxel_size, z*voxel_size])

    #If the voxel exists in the one we created from the original point cloud, color it and add it to the new voxel grid
    #If the voxel does not exist in the original voxel grid, color it black and add it to the new voxel grid
    #The colors is how the AI will distinguish voxels that represent points and voxels that represent empty space
    bools = v.check_if_included(o3d.utility.Vector3dVector(xyz))
    for i in range(len(bools)):
        if(bools[i]):
            fullPoints.append(xyz[i])
            fullColors.append([.5, .5, .5]) # could probably figure out how to get the correct colors into the full voxel grid
        else:
            fullPoints.append(xyz[i])
            fullColors.append([0, 0, 0])

    fullpcd = o3d.geometry.PointCloud()
    fullpcd.points = o3d.utility.Vector3dVector(fullPoints)
    fullpcd.colors = o3d.utility.Vector3dVector(fullColors)
    mins = np.array([0, 0, 0])      #min and max could be changed to function arguments for more manuverability
    maxs = np.array([bound, bound, bound])
    full_voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud_within_bounds(fullpcd, voxel_size, mins, maxs)

    return (full_voxel_grid, fullColors)

def visualize_voxel_grid(voxel_grid : o3d.geometry.VoxelGrid):
    # for v in full_voxel_grid.get_voxels():
    #     print(v.grid_index)
    #print('min: ', voxel_grid.get_min_bound())
    #print('max: ', voxel_grid.get_max_bound())

    # Initialize a visualizer object
    vis = o3d.visualization.Visualizer()
    # Create a window, name it and scale it
    vis.create_window(window_name='Open 3D visualizer', width=800, height=600)

    # Add the voxel grid to the visualizer
    vis.add_geometry(voxel_grid)

    # run the visualizater
    vis.run()
    # Once the visualizer is closed destroy the window and clean up
    vis.destroy_window()

def capture_voxel_grid(voxel_grid : o3d.geometry.VoxelGrid, i : int, save_folder: str):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(voxel_grid)
    vis.update_geometry(voxel_grid)
    vis.poll_events()
    vis.update_renderer()
    save_path = os.path.join(save_folder, "train_%04d.jpg" % i) #change between test/train based on what you're doing
    vis.capture_screen_image(save_path, True)
    print("Saved", save_path)
    vis.destroy_window()

def get_tree(xmin : float, ymin : float, xmax : float, ymax : float, las_file : str):
    # Get an area encompassing the bounding box into an array -> less points to search through
    # Increase this area every iteration of a loop to fully encompass a singular tree
    # Find the maximum value within the bounding box and "pour" from there

    las = laspy.read(las_file)
    point_data = np.stack([las.x, las.y, las.z, las.classification], axis = 0).transpose((1,0))

    # How lenient the distance can be for points to be considered 'close' to another
    x_flex = 5
    y_flex = 5
    z_flex = 5

    # Holds every tree point within bounding box
    x_data = []
    y_data = []
    z_data = []
    colors = []
    zmax = 0

    # Holds every point within the single tree
    x_tree = [0]
    y_tree = [0]
    z_tree = [0]

    # Load all tree points within bound into an array (needs to be modified for unclassified data)
    for i in point_data:
        if i[0] > xmin and i[0] <  xmax and i[1] > ymin and i[1] < ymax and (i[3] == 4 or i[3] == 5):
            x_data.append(i[0])
            y_data.append(i[1])
            z_data.append(i[2])

    # Get the highest tree point in bound, assign that point to the first spot in the tree array
    for i in range(len(z_data)):
        if z_data[i] > zmax:
            zmax = z_data[i]
            x_tree[0] = x_data[i]
            y_tree[0] = y_data[i]
            z_tree[0] = z_data[i]

    print("Tree found -> Coordinates: (", x_tree[0], ",", y_tree[0], ")")

    # INITIAL NEIGHBORS: Add points close below the maximum to the tree array
    for i in range(len(z_data)):
        if ((x_data[i] < x_tree[0] + x_flex) and (x_data[i] > x_tree[0] - x_flex)) and ((y_data[i] < y_tree[0] + y_flex) and (y_data[i] > y_tree[0] - y_flex)) and (z_data[i] < z_tree[0]):
            x_tree.append(x_data[i])
            y_tree.append(y_data[i])
            z_tree.append(z_data[i])

    # Sort the tree arrays for ascending z values (make sure to connect all arrays to the z array)
    # Find the 4 points with the largest z values in the tree and add the points close below them to the tree array (run the algorithm)
    # (xmax, ymax), (xmin, ymin), (xmax, ymin), (xmin, ymax)

    # Sort starts here (Max is at z_tree[0]), sorted in descending order
    x_tree = np.array(x_tree)
    y_tree = np.array(y_tree)
    z_tree = np.array(z_tree)
    idx = np.flip(np.argsort(z_tree))
    x_tree = np.array(x_tree)[idx]
    y_tree = np.array(y_tree)[idx]
    z_tree = np.array(z_tree)[idx]

    # Algorithm starts here
    for i in range(len(z_data)):
        for j in range(20, len(z_tree)-20, 20):
            if ((x_data[i] < x_tree[j] + x_flex) and (x_data[i] > x_tree[j] - x_flex)) and ((y_data[i] < y_tree[j] + y_flex) and (y_data[i] > y_tree[j] - y_flex)) and (z_data[i] < z_tree[j]): #and (z_data[i] > z_tree[j] - z_flex)):
                # If point not already in tree
                if (x_data[i] not in x_tree) or (y_data[i] not in y_tree) or (z_data[i] not in z_tree):
                    x_tree = np.append(x_tree, x_data[i]) #x_tree.append(x_data[i])
                    y_tree = np.append(y_tree, y_data[i]) #y_tree.append(y_data[i])
                    z_tree = np.append(z_tree, z_data[i]) #z_tree.append(z_data[i])

    # Sort starts here
    idx = np.flip(np.argsort(z_tree))
    x_tree = np.array(x_tree)[idx]
    y_tree = np.array(y_tree)[idx]
    z_tree = np.array(z_tree)[idx]

    print("Points in tree:", len(z_tree))
    #print("z_tree[0]:", z_tree[0])
    #print("z_tree[len(z_tree)-1]:", z_tree[len(z_tree)-1])

    # Visualization starts here (temporary)

    # Color the tree 
    for i in range(len(x_tree)):
        colors.append([0.137, 0.922, 0.216])

    # Voxelize points for display
    points = []
    for i in range(len(x_tree)):
        points.append([x_tree[i] - xmin, y_tree[i] - ymin, z_tree[i] - min(z_tree)])
    
    if((max(x_tree) - min(x_tree)) > (max(y_tree) - min(y_tree))):
        bound = max(x_tree) - min(x_tree)
    else:
        bound = max(y_tree) - min(y_tree)
    voxel = voxelize(points, colors, .75, bound)

    return voxel

    

    
