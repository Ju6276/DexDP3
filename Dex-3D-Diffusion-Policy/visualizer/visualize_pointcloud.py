import open3d as o3d
import numpy as np
import os
import argparse
from termcolor import cprint

def load_and_visualize_point_cloud(file_path, visualize=True):
    """
    Load and visualize point cloud file
    Supports .ply, .pcd, .npy formats
    Output format:
    - With color: points is (N, 6) array containing [x, y, z, r, g, b]
    - Without color: points is (N, 3) array containing [x, y, z]
    """
    file_extension = os.path.splitext(file_path)[1]
    
    try:
        if file_extension in ['.ply', '.pcd']:
            point_cloud = o3d.io.read_point_cloud(file_path)
            points = np.asarray(point_cloud.points)
            # Get color information if available
            colors = np.asarray(point_cloud.colors)
            if len(colors) == 0:
                # If no color information, fill with white
                colors = np.zeros((len(points), 3))
        elif file_extension == '.npy':
            data = np.load(file_path)
            # Check data format
            if data.shape[1] == 6:  # If contains color information
                points = data[:, :3]
                colors = data[:, 3:] / 255.0  # Assuming color values are in range 0-255
            else:  # Position information only
                points = data
                colors = np.zeros((len(points), 3))
            
            point_cloud = o3d.geometry.PointCloud()
            point_cloud.points = o3d.utility.Vector3dVector(points)
            point_cloud.colors = o3d.utility.Vector3dVector(colors)

        # Print point cloud information
        cprint(f"Loaded point cloud from: {file_path}", "green")
        cprint(f"Number of points: {len(points)}", "green")
        cprint(f"Point cloud bounds:", "green")
        cprint(f"X: [{points[:, 0].min():.3f}, {points[:, 0].max():.3f}]", "cyan")
        cprint(f"Y: [{points[:, 1].min():.3f}, {points[:, 1].max():.3f}]", "cyan")
        cprint(f"Z: [{points[:, 2].min():.3f}, {points[:, 2].max():.3f}]", "cyan")

        # Determine output format based on color information
        if np.all(colors == 0):  # If all white (no color information)
            output_points = points
            cprint(f"Point cloud format: Array of shape {output_points.shape}", "green")
            cprint(f"Format: [x, y, z]", "green")
        else:
            output_points = np.concatenate([points, colors * 255], axis=1)  # Convert back to 0-255 range
            cprint(f"Point cloud format: Array of shape {output_points.shape}", "green")
            cprint(f"Format: [x, y, z, r, g, b]", "green")

        if visualize:
            # Create coordinate system visualization
            coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
                size=0.1, origin=[0, 0, 0])
            
            # Visualize point cloud and coordinate system
            o3d.visualization.draw_geometries([point_cloud, coordinate_frame],
                                           window_name="Point Cloud Visualization",
                                           width=1024,
                                           height=768)
        
        return point_cloud, output_points

    except Exception as e:
        cprint(f"Error loading point cloud: {str(e)}", "red")
        return None, None

def visualize_multiple_point_clouds(directory_path, file_pattern=None):
    """
    Visualize all point cloud files in a directory
    Can specify file pattern (e.g., 'cloud_*.ply')
    """
    import glob
    
    if file_pattern:
        files = glob.glob(os.path.join(directory_path, file_pattern))
    else:
        # Support multiple formats
        extensions = ['.ply', '.pcd', '.npy']
        files = []
        for ext in extensions:
            files.extend(glob.glob(os.path.join(directory_path, f'*{ext}')))
    
    files.sort()  # Sort by filename
    
    if not files:
        cprint(f"No point cloud files found in {directory_path}", "red")
        return
    
    cprint(f"Found {len(files)} point cloud files", "green")
    
    for file_path in files:
        cprint(f"\nProcessing: {os.path.basename(file_path)}", "yellow")
        load_and_visualize_point_cloud(file_path)

def main():
    parser = argparse.ArgumentParser(description='Point Cloud Visualization Tool')
    parser.add_argument('--path', type=str, required=True,
                       help='Path to point cloud file or directory')
    parser.add_argument('--pattern', type=str, default=None,
                       help='File pattern to match (e.g., "cloud_*.ply")')
    parser.add_argument('--no-vis', action='store_true',
                       help='Disable visualization (only print info)')
    
    args = parser.parse_args()
    
    if os.path.isfile(args.path):
        # Single file
        load_and_visualize_point_cloud(args.path, not args.no_vis)
    elif os.path.isdir(args.path):
        # Directory
        visualize_multiple_point_clouds(args.path, args.pattern)
    else:
        cprint(f"Invalid path: {args.path}", "red")

if __name__ == "__main__":
    main()