import sys
import os
import pathlib
import hydra
import torch
import dill
from omegaconf import OmegaConf
import numpy as np
from termcolor import cprint
import rospy
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray
import message_filters
from cv_bridge import CvBridge
import open3d as o3d
from queue import Queue
import pytorch3d.ops as torch3d_ops
from diffusion_policy_3d.policy.simple_dp3 import SimpleDP3
import time
import threading

'''
Deploy the policy on the robot
''' 

class PointCloudProcessor:
    """Point Cloud Processor"""
    def __init__(self):
        # Camera intrinsics
        self.camera_intrinsics = o3d.camera.PinholeCameraIntrinsic(
            height=480,
            width=640,
            fx=597.031494140625,
            fy=597.585205078125,
            cx=325.9665222167969,
            cy=237.9289855957031
        )
        
        # Extrinsic matrix
        self.extrinsic_matrix = np.array([
            [0.99980397, -0.00226252, 0.01966969, 0.03397305],
            [0.0152679, -0.54440548, -0.83868323, 0.37113355],
            [0.01260582, 0.83881914, -0.54426421, 1.23867239],
            [0., 0., 0., 1.]
        ])
        
        # Workspace range
        self.WORK_SPACE = {
            'x': (-0.25, 0.25),
            'y': (0, 0.85),
            'z': (0.05, 1.2) # raw data=0.05
        }
    
    def process_rgbd(self, color_img, depth_img):
        """Process RGB-D image to generate point cloud"""
        # Convert to Open3D format
        o3d_color = o3d.geometry.Image(color_img)
        o3d_depth = o3d.geometry.Image(depth_img)
        
        # Create RGBD image
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d_color, 
            o3d_depth,
            depth_scale=1000.0,
            depth_trunc=3.0,
            convert_rgb_to_intensity=False
        )
        
        # Generate point cloud
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image, 
            self.camera_intrinsics
        )
        
        # Apply extrinsic transformation
        points = np.asarray(pcd.points)
        points_homogeneous = np.hstack((points, np.ones((points.shape[0], 1))))
        transformed_points = np.dot(points_homogeneous, 
                                  np.linalg.inv(self.extrinsic_matrix).T)[:, :3]
        
        # Workspace filtering
        mask = (
            (transformed_points[:, 0] >= self.WORK_SPACE['x'][0]) & 
            (transformed_points[:, 0] <= self.WORK_SPACE['x'][1]) &
            (transformed_points[:, 1] >= self.WORK_SPACE['y'][0]) & 
            (transformed_points[:, 1] <= self.WORK_SPACE['y'][1]) &
            (transformed_points[:, 2] >= self.WORK_SPACE['z'][0]) & 
            (transformed_points[:, 2] <= self.WORK_SPACE['z'][1])
        )
        filtered_points = transformed_points[mask]
        
        # Farthest point sampling
        if len(filtered_points) > 0:
            filtered_points = torch.from_numpy(filtered_points).cuda()
            sampled_points, _ = torch3d_ops.sample_farthest_points(
                points=filtered_points.unsqueeze(0), 
                K=[1024]
            )
            sampled_points = sampled_points.squeeze(0).cpu().numpy()
            return sampled_points
        return None

class DeployDP3Workspace:
    """Model loading and management"""
    def __init__(self, cfg: OmegaConf):
        self.cfg = cfg
        self.model: SimpleDP3 = hydra.utils.instantiate(cfg.policy)
        self.ema_model: SimpleDP3 = None
        if cfg.training.use_ema:
            self.ema_model = hydra.utils.instantiate(cfg.policy)
            
    def load_checkpoint(self, path):
        payload = torch.load(path, pickle_module=dill, map_location='cpu')
        if 'state_dicts' in payload:
            if 'model' in payload['state_dicts']:
                self.model.load_state_dict(payload['state_dicts']['model'])
            if self.ema_model is not None and 'ema_model' in payload['state_dicts']:
                self.ema_model.load_state_dict(payload['state_dicts']['ema_model'])
        return True

class RealDexPushEnv:
    """Environment management"""
    def __init__(self, cfg):
        rospy.init_node('realdex_push_policy', anonymous=True)
        
        self.cfg = cfg
        self.device = torch.device(cfg.training.device)
        self.n_obs_steps = cfg.n_obs_steps
        self.n_action_steps = cfg.n_action_steps
        
        # Initialize counters and timestamps
        self.robot_state_count = 0
        self.last_robot_state_time = None
        
        # Initialize data cache
        self.point_cloud_array = []
        self.robot_state_array = []
        
        # Initialize point cloud processor
        self.pc_processor = PointCloudProcessor()
        self.bridge = CvBridge()
        
        # Subscribe to images and depth
        self.image_sub = message_filters.Subscriber(
            '/camera/color/image_raw', Image)
        self.depth_sub = message_filters.Subscriber(
            '/camera/aligned_depth_to_color/image_raw', Image)
        
        # Synchronize subscription
        ts = message_filters.TimeSynchronizer(
            [self.image_sub, self.depth_sub], 10)
        ts.registerCallback(self.rgbd_callback)
        
        # Subscribe to robot state
        self.robot_state_sub = rospy.Subscriber(
            '/combined_robot_state',
            JointState,
            self.robot_state_callback,
            queue_size=1
        )
        
        # Publish action
        self.action_pub = rospy.Publisher(
            '/predicted_action',
            JointState,
            queue_size=1
        )
        
        # Data queue
        self.rgbd_queue = Queue(maxsize=1)
        self.robot_state_queue = Queue(maxsize=1)
        
        # Add execution completion flag
        self.action_completed = threading.Event()
        
        # Add execution completion subscription
        rospy.Subscriber("/execution_done", JointState, self.execution_done_callback)
    
    def execution_done_callback(self, msg):
        """Handle action execution completion callback"""
        self.action_completed.set()
        rospy.loginfo("Received action execution completion signal")
    
    def rgbd_callback(self, img_msg, depth_msg):
        """RGB-D data callback"""
        try:
            color_img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
            
            if self.rgbd_queue.full():
                self.rgbd_queue.get()
            self.rgbd_queue.put((color_img, depth_img))
        except Exception as e:
            rospy.logerr(f"Error in RGBD callback: {e}")
    
    def robot_state_callback(self, msg):
        """Robot state callback"""
        try:
            robot_state = np.array(msg.position)
            self.robot_state_count += 1
            self.last_robot_state_time = time.time()
            
            if self.robot_state_queue.full():
                self.robot_state_queue.get()
            self.robot_state_queue.put(robot_state)
            
        except Exception as e:
            rospy.logerr(f"Error in robot state callback: {e}")
    
    def reset(self):
        """Reset environment state"""
        self.point_cloud_array = []
        self.robot_state_array = []
        
        # Wait for enough observation data
        while len(self.robot_state_array) < self.n_obs_steps:
            if not self.rgbd_queue.empty() and not self.robot_state_queue.empty():
                color_img, depth_img = self.rgbd_queue.get()
                robot_state = self.robot_state_queue.get()
                
                # Process point cloud
                point_cloud = self.pc_processor.process_rgbd(color_img, depth_img)
                if point_cloud is not None:
                    self.point_cloud_array.append(point_cloud)
                    self.robot_state_array.append(robot_state)
            rospy.sleep(0.01)
        
        return self._get_obs()
    
    def step(self, action):
        """Publish action and wait for execution"""
        try:
            # Clear previous completion flag
            self.action_completed.clear()
            
            # Create JointState message
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            
            # Ensure conversion to numpy array
            if isinstance(action, torch.Tensor):
                action = action.cpu().numpy()
            
            # Flatten if nested structure
            if isinstance(action, np.ndarray):
                action = action.flatten()
            
            # Print debug information
            rospy.loginfo("="*50)
            rospy.loginfo("Sending new action command:")
            rospy.loginfo(f"Position: [{action[0]:.4f}, {action[1]:.4f}, {action[2]:.4f}]")
            
            # Publish action
            msg.position = action.astype(np.float64).tolist()
            self.action_pub.publish(msg)
            
            # Wait for action execution
            if not self.action_completed.wait(timeout=10.0):  # 10 seconds timeout
                rospy.logwarn("Timeout waiting for action execution")
                return self._get_obs()
            
            rospy.loginfo("Action execution completed, getting new observation")
            
            # Wait for new observation data
            if not self.rgbd_queue.empty() and not self.robot_state_queue.empty():
                color_img, depth_img = self.rgbd_queue.get()
                robot_state = self.robot_state_queue.get()
                
                # Process point cloud
                point_cloud = self.pc_processor.process_rgbd(color_img, depth_img)
                if point_cloud is not None:
                    self.point_cloud_array.append(point_cloud)
                    self.robot_state_array.append(robot_state)
                
                # Maintain history length
                if len(self.point_cloud_array) > self.n_obs_steps:
                    self.point_cloud_array.pop(0)
                if len(self.robot_state_array) > self.n_obs_steps:
                    self.robot_state_array.pop(0)
            
            return self._get_obs()
            
        except Exception as e:
            rospy.logerr(f"Error in action execution: {e}")
            rospy.logerr(f"Action type: {type(action)}, content: {action}")
            raise
    
    def _get_obs(self):
        """Build observation dictionary"""
        obs_dict = {
            'point_cloud': torch.from_numpy(
                np.stack(self.point_cloud_array[-self.n_obs_steps:], axis=0)
            ).unsqueeze(0).to(self.device),
            'agent_pos': torch.from_numpy(
                np.stack(self.robot_state_array[-self.n_obs_steps:], axis=0)
            ).unsqueeze(0).to(self.device)
        }
        return obs_dict

@hydra.main(
    version_base=None,
    config_path=str(pathlib.Path(__file__).parent.joinpath(
        'diffusion_policy_3d', 'config'))
)
def main(cfg):
    # Initialize workspace
    workspace = DeployDP3Workspace(cfg)
    
    # Load checkpoint
    ckpt_path = os.path.join(cfg.run_dir, 'checkpoints/latest.ckpt')
    if not workspace.load_checkpoint(ckpt_path):
        raise ValueError(f"Failed to load checkpoint from {ckpt_path}")
    
    # Select model to use
    policy = workspace.ema_model if workspace.ema_model is not None else workspace.model
    policy.eval()
    policy.to(cfg.training.device)
    
    # Initialize environment
    env = RealDexPushEnv(cfg)
    
    try:
        obs = env.reset()
        rate = rospy.Rate(10)  # 10Hz control frequency
        
        while not rospy.is_shutdown():
            with torch.no_grad():
                # Predict action
                result = policy.predict_action(obs)
                action = result['action_pred']
                
                # Print raw action shape
                rospy.loginfo(f"Raw action shape: {action.shape}")
                
                # Reshape action: assume shape is [1, 4, 23], take only the first time step
                if len(action.shape) == 3:
                    action = action[0, 0]  # First batch's first time step
                elif len(action.shape) == 2:
                    action = action[0]  # Take only the first batch
                
                # Ensure action is correct dimension (23D)
                action = action.cpu().numpy()
                if len(action) > 23:
                    action = action[:23]  # Take only the first 23D
                
                rospy.loginfo(f"Final action shape: {action.shape}")
                rospy.loginfo(f"Final action values: {action[:7]}")
                
                # Publish action and wait for execution
                obs = env.step(action)
                
                rate.sleep()
                
    except KeyboardInterrupt:
        rospy.loginfo("Policy node terminated by user")

if __name__ == "__main__":
    main()