import numpy as np
import cv2
import open3d as o3d
import os
from tqdm import tqdm
import torch
import pytorch3d.ops as torch3d_ops

'''
This script is used to generate point clouds from the raw data.
''' 

def farthest_point_sampling(points, num_points=1024, use_cuda=True):
    """Perform farthest point sampling on point cloud"""
    K = [num_points]
    if use_cuda:
        points = torch.from_numpy(points).cuda()
        sampled_points, indices = torch3d_ops.sample_farthest_points(points=points.unsqueeze(0), K=K)
        sampled_points = sampled_points.squeeze(0)
        sampled_points = sampled_points.cpu().numpy()
    else:
        points = torch.from_numpy(points)
        sampled_points, indices = torch3d_ops.sample_farthest_points(points=points.unsqueeze(0), K=K)
        sampled_points = sampled_points.squeeze(0)
        sampled_points = sampled_points.numpy()

    return sampled_points, indices

def transform_point_cloud(pcd, extrinsic_matrix):
    """Transform point cloud using inverse of extrinsic matrix"""
    extrinsic_matrix_inv = np.linalg.inv(extrinsic_matrix)
    points = np.asarray(pcd.points)
    points_homogeneous = np.hstack((points, np.ones((points.shape[0], 1))))
    transformed_points = np.dot(points_homogeneous, extrinsic_matrix_inv.T)
    transformed_points = transformed_points[:, :3]
    
    transformed_pcd = o3d.geometry.PointCloud()
    transformed_pcd.points = o3d.utility.Vector3dVector(transformed_points)
    if pcd.has_colors():
        transformed_pcd.colors = pcd.colors
    return transformed_pcd

def process_episode(episode_dir, camera_intrinsics, extrinsic_matrix):
    """Process data for a single episode"""
    img_dir = os.path.join(episode_dir, "img")
    depth_dir = os.path.join(episode_dir, "depth")
    point_cloud_dir = os.path.join(episode_dir, "point_cloud")
    os.makedirs(point_cloud_dir, exist_ok=True)

    # Get all frame numbers
    frames = sorted([f[6:-4] for f in os.listdir(img_dir) if f.startswith('frame_') and f.endswith('.png')])
    
    for frame_idx in tqdm(frames, desc=f"Processing {os.path.basename(episode_dir)}"):
        color_path = os.path.join(img_dir, f"frame_{frame_idx}.png")
        depth_path = os.path.join(depth_dir, f"frame_{frame_idx}.npy")
        
        # Read image and depth data
        color_image = cv2.imread(color_path)
        depth_image = np.load(depth_path)
        
        # Convert to Open3D format
        o3d_color = o3d.geometry.Image(color_image)
        o3d_depth = o3d.geometry.Image(depth_image)

        # Depth scale, adjust according to your data
        depth_scale = 0.001

        # Create RGBD image
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d_color, 
            o3d_depth,
            depth_scale=1.0 / depth_scale,
            depth_trunc=3.0,
            convert_rgb_to_intensity=False
        )

        # Generate point cloud
        point_cloud = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image, 
            camera_intrinsics
        )
        
        # Apply extrinsic matrix transformation
        transformed_cloud = transform_point_cloud(point_cloud, extrinsic_matrix)

        # Define workspace boundaries
        WORK_SPACE = {
            'x': (-0.25, 0.25),      # Left-right range
            'y': (0, 0.85),          # Up-down range
            'z': (0.05, 1.2)         # Front-back range
        }

        # Filter point cloud
        points = np.asarray(transformed_cloud.points)
        mask = (
            (points[:, 0] >= WORK_SPACE['x'][0]) & (points[:, 0] <= WORK_SPACE['x'][1]) &
            (points[:, 1] >= WORK_SPACE['y'][0]) & (points[:, 1] <= WORK_SPACE['y'][1]) &
            (points[:, 2] >= WORK_SPACE['z'][0]) & (points[:, 2] <= WORK_SPACE['z'][1])
        )
        filtered_points = points[mask]

        # Add farthest point sampling
        if len(filtered_points) > 0:
            sampled_points, _ = farthest_point_sampling(filtered_points, num_points=1024)
            filtered_points = sampled_points

            # Save point cloud
            pcd_path = os.path.join(point_cloud_dir, f"frame_{frame_idx}")
            np.save(f"{pcd_path}.npy", filtered_points)

def process_all_data(data_root, camera_intrinsics, extrinsic_matrix):
    """Process data for all episodes"""
    episodes = sorted([d for d in os.listdir(data_root) if d.startswith('episode_')])
    total_episodes = len(episodes)
    print(f"\nFound {total_episodes} episodes")
    
    for episode_idx, episode in enumerate(episodes, 1):
        episode_dir = os.path.join(data_root, episode)
        if os.path.isdir(episode_dir):
            print(f"\nProcessing {episode} ({episode_idx}/{total_episodes})...")
            process_episode(episode_dir, camera_intrinsics, extrinsic_matrix)

def main():
    data_root = "/home/ju/Downloads/data_raw2"
    
    # Set camera intrinsics
    camera_intrinsics = o3d.camera.PinholeCameraIntrinsic(
        height=480,
        width=640,
        fx=597.031494140625,
        fy=597.585205078125,
        cx=325.9665222167969,
        cy=237.9289855957031
    )

    # Set extrinsic matrix
    EXTRINSIC_MATRIX = np.array([
        [0.99980397, -0.00226252, 0.01966969, 0.03397305],
        [0.0152679, -0.54440548, -0.83868323, 0.37113355],
        [0.01260582, 0.83881914, -0.54426421, 1.23867239],
        [0., 0., 0., 1.]
    ])

    # Process all data
    process_all_data(data_root, camera_intrinsics, EXTRINSIC_MATRIX)

if __name__ == "__main__":
    main()