#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import threading

'''
This is for publishing the end-effector pose and Allegro hand joint states
'''
class CombinedStatePublisher:
    def __init__(self):
        rospy.init_node('combined_state_publisher')
        
        # Set publishing frequency
        self.publish_rate = rospy.Rate(30)  # Set to 30Hz
        
        # Store latest states
        self.allegro_states = None
        self.ee_pose = None
        self.state_lock = threading.Lock()
        
        # Create publisher - publishes combined joint states
        self.pub = rospy.Publisher('/combined_robot_state', JointState, queue_size=10)
        
        # Subscribers
        self.sub_allegro = rospy.Subscriber(
            '/allegroHand/joint_states', 
            JointState, 
            self.allegro_cb,
            queue_size=10
        )
        self.sub_ee_pose = rospy.Subscriber(
            'end_effector_actual_pose', 
            JointState, 
            self.ee_pose_cb,
            queue_size=10
        )
        
        # Start publishing thread
        self.publish_thread = threading.Thread(target=self.publish_loop)
        self.publish_thread.daemon = True
        self.publish_thread.start()
        
        rospy.loginfo("Combined state publisher initialized")

    def allegro_cb(self, msg):
        """Callback for Allegro hand joint states"""
        with self.state_lock:
            self.allegro_states = msg

    def ee_pose_cb(self, msg):
        """Callback for end-effector pose"""
        with self.state_lock:
            self.ee_pose = msg

    def publish_loop(self):
        """Publish states at fixed frequency"""
        while not rospy.is_shutdown():
            self.publish_combined_states()
            self.publish_rate.sleep()

    def publish_combined_states(self):
        """Publish combined states"""
        with self.state_lock:
            if not all([self.allegro_states, self.ee_pose]):
                return

            # Create new joint state message
            combined_msg = JointState()
            combined_msg.header.stamp = rospy.Time.now()
            combined_msg.header.frame_id = "world"
            
            # Combine names and position data
            combined_msg.name = list(self.ee_pose.name) + list(self.allegro_states.name)
            combined_msg.position = list(self.ee_pose.position) + list(self.allegro_states.position)

            # Publish combined message
            self.pub.publish(combined_msg)

if __name__ == '__main__':
    try:
        node = CombinedStatePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass