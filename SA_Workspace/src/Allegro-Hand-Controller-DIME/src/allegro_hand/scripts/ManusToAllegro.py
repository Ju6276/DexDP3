#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
import math

class GloveToAllegro:
    def __init__(self):
        rospy.init_node('glove_to_allegro', anonymous=True)

        # Subscribe to /manus_glove_data_left data (std_msgs/Float32MultiArray)
        rospy.Subscriber('/manus_glove_data_left', Float32MultiArray, self.callback)

        # Publish to AllegroHand controller
        self.pub = rospy.Publisher('/allegroHand/joint_cmd', JointState, queue_size=10)

        # Initialize joint state, set joint names and initial values
        self.joint_state = JointState()
        self.joint_state.name = [
            'joint_0.0', 'joint_1.0', 'joint_2.0', 'joint_3.0',  # Index finger
            'joint_4.0', 'joint_5.0', 'joint_6.0', 'joint_7.0',  # Middle finger
            'joint_8.0', 'joint_9.0', 'joint_10.0', 'joint_11.0',  # Ring finger
            'joint_12.0', 'joint_13.0', 'joint_14.0', 'joint_15.0'  # Thumb
        ]
        self.joint_state.position = [0.0] * 16  # Set initial values

        # Define joint limits
        self.joint_limits = {
            "joint_0.0": (-0.47, 0.47),
            "joint_1.0": (-0.196, 1.61),
            "joint_2.0": (-0.174, 1.709),
            "joint_3.0": (-0.227, 1.618),
            "joint_4.0": (-0.47, 0.47),
            "joint_5.0": (-0.196, 1.61),
            "joint_6.0": (-0.174, 1.709),
            "joint_7.0": (-0.227, 1.618),
            "joint_8.0": (-0.47, 0.47),
            "joint_9.0": (-0.196, 1.61),
            "joint_10.0": (-0.174, 1.709),
            "joint_11.0": (-0.227, 1.618),
            "joint_12.0": (0.263, 1.396),
            "joint_13.0": (-0.105, 1.163),
            "joint_14.0": (-0.189, 1.644),
            "joint_15.0": (-0.162, 1.719)
        }

        rospy.spin()

    def callback(self, msg):
        """
        Convert glove data to radians and constrain within URDF joint limits.
        """
        glove_data = msg.data

        # Check if data length is as expected
        if len(glove_data) >= 16:
            # Reorder glove data order (manus and allegro joint matching)
            reordered_data = self.reorder_glove_data(glove_data)

            # Convert to radians
            radians_data = [math.radians(angle) for angle in reordered_data]

            # Apply joint limits
            constrained_data = [
                self.apply_joint_limits(joint_name, radians)
                for joint_name, radians in zip(self.joint_state.name, radians_data)
            ]

            # Update joint state
            self.joint_state.position = constrained_data
            self.joint_state.header.stamp = rospy.Time.now()

            # Publish joint command
            self.pub.publish(self.joint_state)
        else:
            rospy.logwarn("Received glove data with insufficient length!")

    def reorder_glove_data(self, glove_data):
        """
        Reorder glove data to match AllegroHand joint order:
        Original order: Thumb, Index, Middle, Ring
        Target order: Index, Middle, Ring, Thumb
        """
        # Split glove data into original order
        thumb = glove_data[0:4]  # Thumb
        index = glove_data[4:8]  # Index
        middle = glove_data[8:12]  # Middle
        ring = glove_data[12:16]  # Ring
        # Return reordered data
        return index + middle + ring + thumb

    def apply_joint_limits(self, joint_name, value):
        """
        Constrain value based on joint name and joint limits.
        :param joint_name: Joint name
        :param value: Value to constrain
        :return: Constrained value
        """
        if joint_name in self.joint_limits:
            lower, upper = self.joint_limits[joint_name]
            return max(min(value, upper), lower)
        else:
            rospy.logwarn(f"Joint {joint_name} has no limits defined!")
            return value


if __name__ == '__main__':
    try:
        GloveToAllegro()
    except rospy.ROSInterruptException:
        pass
