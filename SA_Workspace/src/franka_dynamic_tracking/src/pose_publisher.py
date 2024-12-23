#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf
import tf2_geometry_msgs
'''
This is for publish the end-effector pose to the joint_state topic
'''
class EndEffectorPosePublisher:
    def __init__(self):
        rospy.init_node('end_effector_pose_publisher')
        
        # Create pose publisher
        self.actual_pose_pub = rospy.Publisher(
            "end_effector_actual_pose", 
            JointState, 
            queue_size=1
        )
        
        # Initialize TF listener (supports both tf and tf2)
        self.tf_listener = tf.TransformListener()
        
        # Wait for TF tree to be ready
        rospy.sleep(1.0)  # Wait 1 second to ensure TF tree is established
        
        # Set publishing frequency
        self.rate = rospy.Rate(100)  # 100Hz
        
        rospy.loginfo("End-effector pose publisher initialized")
        
        # Start publishing pose
        self.publish_pose_loop()

    def publish_pose_loop(self):
        """Loop to publish end-effector pose"""
        while not rospy.is_shutdown():
            try:
                # Get transform
                (trans, rot) = self.tf_listener.lookupTransform(
                    'panda_link0',    
                    'franka_flange',  
                    rospy.Time(0)
                )
                
                # Create PoseStamped message
                current_pose = PoseStamped()
                current_pose.header.frame_id = "panda_link0"
                current_pose.header.stamp = rospy.Time.now()
                
                # Set position and orientation
                current_pose.pose.position.x = trans[0]
                current_pose.pose.position.y = trans[1]
                current_pose.pose.position.z = trans[2]
                current_pose.pose.orientation.x = rot[0]
                current_pose.pose.orientation.y = rot[1]
                current_pose.pose.orientation.z = rot[2]
                current_pose.pose.orientation.w = rot[3]
                
                # Convert to JointState format and publish
                self.publish_pose(current_pose)
                
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException) as e:
                rospy.logwarn_throttle(1, f"获取TF转换失败: {str(e)}")
                
            self.rate.sleep()

    def publish_pose(self, pose):
        """Publish pose information"""
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.header.frame_id = "panda_link0"
        
        # Define joint names
        joint_state.name = ['ee_x', 'ee_y', 'ee_z', 'ee_qx', 'ee_qy', 'ee_qz', 'ee_qw']
        
        # Set position values
        joint_state.position = [
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
        ]
        
        # Publish message
        self.actual_pose_pub.publish(joint_state)

def main():
    try:
        publisher = EndEffectorPosePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()