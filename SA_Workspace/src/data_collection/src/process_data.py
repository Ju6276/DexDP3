#!/usr/bin/env python3
import rospy
import numpy as np
import message_filters
from sensor_msgs.msg import Image, CameraInfo, JointState
import os
from cv_bridge import CvBridge
import argparse
import cv2
from datetime import datetime
'''
This script is used to collect data from the robot. 
The data is saved in the root_dir directory.
The data is saved in the following format:
    - img: the color image
    - depth: the depth image
    - state: the state of the robot
    - action: the action of the robot
    - timestamps: the timestamps of the data
''' 
def parse_args():
    parser = argparse.ArgumentParser()
    default_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'data_raw2')
    parser.add_argument('--root_dir', type=str, default=default_path, help='Directory to save data')
    args = parser.parse_args()
    
    os.makedirs(args.root_dir, exist_ok=True)
    return args

class DataCollector:
    def __init__(self, args):
        rospy.init_node('data_collector')
        self.bridge = CvBridge()
        self.root_dir = args.root_dir
        
        # Parameter settings
        self.fps = 5 # 10Hz
        self.last_capture_time = rospy.Time.now().to_sec()  # Initialize time
        self.capture_interval = 1.0 / self.fps  # Calculate sampling interval based on fps

        # Data storage
        self.episode_idx = None
        self.current_episode_dir = None
        
        # State control
        self.is_recording = False
        self.current_frame = 0
        
        # Data storage lists (for temporarily storing data of one episode)
        self.state_arrays = []
        self.action_arrays = []
        self.timestamps = []
        
        # Setup synchronized subscribers and callbacks
        self._setup_subscribers()
        
        self.previous_state = None
        rospy.loginfo("Data collector initialized. Press 's' to start collecting, 'e' to end collection")
        
        # Add keyboard listener
        import threading
        threading.Thread(target=self._keyboard_listener, daemon=True).start()

    def _setup_subscribers(self):
        self.image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        self.depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        self.joint_sub = message_filters.Subscriber('/combined_robot_state', JointState)
        self.camera_info_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/camera_info', CameraInfo)
        self.camera_intrinsics = None
        self.camera_info_sub.registerCallback(self.camera_info_callback)
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub, self.joint_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.sync_callback)

    def _create_episode_directory(self):
        """Create new episode directory"""
        episode_name = f"episode_{self.episode_idx:03d}"
        episode_dir = os.path.join(self.root_dir, episode_name)
        os.makedirs(os.path.join(episode_dir, "img"), exist_ok=True)
        os.makedirs(os.path.join(episode_dir, "depth"), exist_ok=True)
        return episode_dir

    def sync_callback(self, img_msg, depth_msg, joint_msg):
        if not self.is_recording:
            return
    
        current_time = rospy.Time.now().to_sec()
        if current_time - self.last_capture_time < self.capture_interval:
            return  # Skip this sample

        self.last_capture_time = current_time  # Update last sampling time
        
        try:
            # Convert image data
            color_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
            rospy.loginfo(f"Color image size: {color_image.shape}, data type: {color_image.dtype}")
            rospy.loginfo(f"Depth image size: {depth_image.shape}, data type: {depth_image.dtype}")
            rospy.loginfo(f"Depth image range: min={np.min(depth_image)}, max={np.max(depth_image)}")
            
            # Get current joint positions as state
            current_state = np.array(joint_msg.position, dtype=np.float32)
            
            # Record current state as both state and action
            self.state_arrays.append(current_state)
            self.action_arrays.append(current_state)  # Modified to record current state
            
            # Save raw image and depth data
            frame_name = f"frame_{self.current_frame:04d}"
            cv2.imwrite(os.path.join(self.current_episode_dir, "img", f"{frame_name}.png"), color_image)
            np.save(os.path.join(self.current_episode_dir, "depth", f"{frame_name}.npy"), depth_image)
            
            # Collect timestamp data
            self.timestamps.append(rospy.Time.now().to_sec())
            
            self.current_frame += 1
            
            if self.current_frame % 10 == 0:
                time_elapsed = self.current_frame / 10.0
                rospy.loginfo(f"Trajectory {self.episode_idx}: Collected {self.current_frame} frames ({time_elapsed:.1f} seconds)")
            
        except Exception as e:
            rospy.logerr(f"Error in sync_callback: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())

    def _start_recording(self):
        """Start recording"""
        while True:
            try:
                idx = input("Please enter the current trajectory index (integer starting from 0): ")
                self.episode_idx = int(idx)
                if self.episode_idx < 0:
                    print("Index must be greater than or equal to 0!")
                    continue
                break
            except ValueError:
                print("Please enter a valid integer!")
        
        self.current_episode_dir = self._create_episode_directory()
        self.is_recording = True
        self.current_frame = 0
        self.state_arrays = []
        self.action_arrays = []
        self.timestamps = []
        self.previous_state = None
        rospy.loginfo(f"Started recording trajectory {self.episode_idx}...")
        rospy.loginfo("Press 'e' to end current trajectory collection")

    def _stop_recording(self):
        """Stop recording"""
        if not self.is_recording:
            return
            
        self.is_recording = False
        if len(self.state_arrays) > 0:
            # Save state, action and timestamp data
            np.save(os.path.join(self.current_episode_dir, "state.npy"), np.array(self.state_arrays))
            np.save(os.path.join(self.current_episode_dir, "action.npy"), np.array(self.action_arrays))
            np.save(os.path.join(self.current_episode_dir, "timestamps.npy"), np.array(self.timestamps))
            
            rospy.loginfo(f"Finished recording trajectory {self.episode_idx}, total {self.current_frame} frames")
            rospy.loginfo(f"Data saved to: {self.current_episode_dir}")
            rospy.loginfo("Press 's' to start collecting new trajectory, press 'q' to quit")
        else:
            rospy.loginfo("No data collected, discarding")

    def _keyboard_listener(self):
        """Listen for keyboard input"""
        while not rospy.is_shutdown():
            key = input().lower()
            if key == 's' and not self.is_recording:
                self._start_recording()
            elif key == 'e' and self.is_recording:
                self._stop_recording()
            elif key == 'q':
                if self.is_recording:
                    self._stop_recording()
                rospy.signal_shutdown("User requested shutdown")
                break

    def camera_info_callback(self, msg):
        """Process camera intrinsic parameters"""
        if self.camera_intrinsics is None:
            self.camera_intrinsics = {
                'fx': msg.K[0],  # focal length x
                'fy': msg.K[4],  # focal length y
                'cx': msg.K[2],  # principal point x
                'cy': msg.K[5]   # principal point y
            }
            rospy.loginfo("Camera intrinsics obtained")

def main():
    args = parse_args()
    collector = DataCollector(args)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        if collector.is_recording:
            collector._stop_recording()
        rospy.loginfo("Data collection stopped")

if __name__ == '__main__':
    main()