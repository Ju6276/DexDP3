#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
import message_filters
from cv_bridge import CvBridge
import numpy as np
import time

'''
Test the network latency of the robot
'''

class ROSNetworkTester:
    def __init__(self):
        rospy.init_node('ros_network_tester', anonymous=True)
        self.bridge = CvBridge()
        
        # Record reception timestamps
        self.last_image_time = None
        self.last_depth_time = None
        self.last_robot_state_time = None
        
        # Data counters
        self.image_count = 0
        self.depth_count = 0
        self.robot_state_count = 0
        
        # Subscribe to camera topics
        self.image_sub = message_filters.Subscriber(
            '/camera/color/image_raw', Image)
        self.depth_sub = message_filters.Subscriber(
            '/camera/aligned_depth_to_color/image_raw', Image)
            
        # Synchronized subscription
        ts = message_filters.TimeSynchronizer(
            [self.image_sub, self.depth_sub], 10)
        ts.registerCallback(self.rgbd_callback)
        
        # Subscribe to robot state
        self.robot_state_sub = rospy.Subscriber(
            '/combined_robot_state',
            Float64MultiArray,
            self.robot_state_callback,
            queue_size=1
        )
        
        # Print status periodically
        rospy.Timer(rospy.Duration(5.0), self.print_status)
        
    def rgbd_callback(self, img_msg, depth_msg):
        """RGB-D data callback"""
        current_time = time.time()
        
        try:
            # Convert images
            color_img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
            
            # Update counters and timestamps
            self.image_count += 1
            self.depth_count += 1
            self.last_image_time = current_time
            self.last_depth_time = current_time
            
            # Print image information
            rospy.loginfo(f"Received RGBD pair - Color shape: {color_img.shape}, "
                         f"Depth shape: {depth_img.shape}")
            
        except Exception as e:
            rospy.logerr(f"Error in RGBD callback: {e}")
    
    def robot_state_callback(self, msg):
        """Robot state callback"""
        current_time = time.time()
        
        try:
            robot_state = np.array(msg.data)
            self.robot_state_count += 1
            self.last_robot_state_time = current_time
            
            rospy.loginfo(f"Received robot state - Shape: {robot_state.shape}, "
                         f"Values: {robot_state[:3]}...")
            
        except Exception as e:
            rospy.logerr(f"Error in robot state callback: {e}")
    
    def print_status(self, event):
        """Print status information"""
        current_time = time.time()
        
        # Calculate delays
        image_delay = (current_time - self.last_image_time) if self.last_image_time else float('inf')
        depth_delay = (current_time - self.last_depth_time) if self.last_depth_time else float('inf')
        robot_delay = (current_time - self.last_robot_state_time) if self.last_robot_state_time else float('inf')
        
        status_msg = f"""
        ====== ROS Network Status ======
        Image Count: {self.image_count} (delay: {image_delay:.3f}s)
        Depth Count: {self.depth_count} (delay: {depth_delay:.3f}s)
        Robot State Count: {self.robot_state_count} (delay: {robot_delay:.3f}s)
        
        ROS Master URI: {rospy.get_master().uri}
        Node Name: {rospy.get_name()}
        ==============================
        """
        rospy.loginfo(status_msg)

def main():
    try:
        tester = ROSNetworkTester()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Network test terminated by user")

if __name__ == "__main__":
    main()
