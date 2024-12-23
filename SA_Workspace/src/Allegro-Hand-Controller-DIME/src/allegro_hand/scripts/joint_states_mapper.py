#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

## Map Allegro hand joint states to FrankaAllegro joint states
## Visualize the full FrankaAllegro model in movelt

class JointStateMapper:
    def __init__(self):
        rospy.init_node('joint_states_mapper')
        
        # Publish to /joint_states
        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        
        # Subscribe to /allegroHand/joint_states
        self.sub = rospy.Subscriber('/allegroHand/joint_states', JointState, self.callback)
        
        # Create mapping dictionary
        self.mapping = {}
        for i in range(16):
            self.mapping[f'joint_{i}.0'] = f'allegro_joint_{i}'

    def callback(self, msg):
        # Create new JointState message
        new_msg = JointState()
        new_msg.header = Header()
        new_msg.header.stamp = rospy.Time.now()
        
        # Map and add allegro joints
        new_msg.name = [self.mapping[old_name] for old_name in msg.name]
        
        # Copy position, velocity, and effort data (if they exist)
        if msg.position:
            new_msg.position = msg.position
            
        if msg.velocity:
            new_msg.velocity = msg.velocity
            
        if msg.effort:
            new_msg.effort = msg.effort
        
        # Publish new message
        self.pub.publish(new_msg)

if __name__ == '__main__':
    try:
        mapper = JointStateMapper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 