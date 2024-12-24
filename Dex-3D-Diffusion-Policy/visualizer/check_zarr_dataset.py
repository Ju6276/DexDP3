#!/usr/bin/env python3
import zarr
import numpy as np
from termcolor import cprint

'''
This script is used to check and display the contents of a zarr dataset.
'''

# Path to zarr file
# zarr_path = '/home/ju/3D-Diffusion-Policy/3D-Diffusion-Policy/data/realdex_push.zarr'
#zarr_path = '/home/ju/catkin_ws/src/data_collection/merged_data.zarr'
zarr_path = '/home/ju/3D-Diffusion-Policy/3D-Diffusion-Policy/data/realdex_grasp.zarr'
try:
    # Open zarr file
    root = zarr.open(zarr_path, mode='r')
    data = root['data']
    
    # Get all arrays
    img_arrays = data['img'] if 'img' in data else None
    point_cloud_arrays = data['point_cloud'] if 'point_cloud' in data else None
    depth_arrays = data['depth'] if 'depth' in data else None
    action_arrays = data['action'] if 'action' in data else None
    state_arrays = data['state'] if 'state' in data else None
    episode_ends = data['episode_ends'] if 'episode_ends' in data else None
    
    # Print dataset information
    cprint("\n Dataset basic information:", "yellow")
    
    # Check and print information for each array
    if img_arrays is not None:
        cprint(f'Image shape: {img_arrays.shape}, range: [{np.min(img_arrays)}, {np.max(img_arrays)}]', 'green')
    
    if point_cloud_arrays is not None:
        cprint(f'Point cloud shape: {point_cloud_arrays.shape}, range: [{np.min(point_cloud_arrays)}, {np.max(point_cloud_arrays)}]', 'green')
    
    if depth_arrays is not None:
        cprint(f'Depth shape: {depth_arrays.shape}, range: [{np.min(depth_arrays)}, {np.max(depth_arrays)}]', 'green')
    
    if action_arrays is not None:
        cprint(f'Action shape: {action_arrays.shape}, range: [{np.min(action_arrays)}, {np.max(action_arrays)}]', 'green')
    
    if state_arrays is not None:
        cprint(f'State shape: {state_arrays.shape}, range: [{np.min(state_arrays)}, {np.max(state_arrays)}]', 'green')
    
    if episode_ends is not None:
        cprint(f'Episode end point shape: {episode_ends.shape}, range: [{np.min(episode_ends)}, {np.max(episode_ends)}]', 'green')
    
    # Calculate total frames
    total_count = img_arrays.shape[0] if img_arrays is not None else (
        point_cloud_arrays.shape[0] if point_cloud_arrays is not None else (
        depth_arrays.shape[0] if depth_arrays is not None else 0
    ))
    cprint(f'Frames: {total_count}', 'green')
    
    # Print dataset structure
    cprint("\n Dataset structure:", "yellow")
    def print_structure(group, prefix=''):
        for key in group.keys():
            if isinstance(group[key], zarr.Group):
                cprint(f"{prefix}├── {key}/", "blue")
                print_structure(group[key], prefix + '│   ')
            else:
                cprint(f"{prefix}├── {key}: {group[key].shape} ({group[key].dtype})", "cyan")
    
    print_structure(root)

except Exception as e:
    cprint(f"Error: {str(e)}", "red")