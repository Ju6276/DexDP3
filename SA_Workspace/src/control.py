#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import moveit_commander
import numpy as np
import threading

'''
    This is the controller for the robot arm and hand.
    It uses MoveIt to plan and execute motions.
    It also subscribes to the predicted action from the training machine.
'''

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller')
        
        # initialize MoveIt
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # set up arm group and end-effector
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.arm_group.set_end_effector_link("franka_flange")
        
        # set up planning parameters(optional)
        # self.arm_group.set_planning_time(5)
        # self.arm_group.set_num_planning_attempts(10)
        # self.arm_group.set_max_velocity_scaling_factor(0.3)
        # self.arm_group.set_goal_position_tolerance(0.01)  # 1cm
        # self.arm_group.set_goal_orientation_tolerance(0.01)  # ~0.57 degree
        
        # get current pose as reference
        current_pose = self.arm_group.get_current_pose().pose
        rospy.loginfo(f"Current pose: \n{current_pose}")
 
        # AllegroHand publisher
        self.hand_pub = rospy.Publisher('/allegroHand/joint_cmd', JointState, queue_size=11)
        
        # initialize Allegro hand settings
        self.setup_allegro_hand()
        
        # subscribe from the training machine's predicted action
        rospy.Subscriber("/predicted_action", JointState, self.action_callback)
        
        rospy.loginfo("Robot controller initialized")
        
        # add action execution status
        self.executing = False
        self.execution_lock = threading.Lock()
        self.action_completed = threading.Event()
        
        # modify publisher, for notifying action execution completion
        self.execution_done_pub = rospy.Publisher('/execution_done', JointState, queue_size=1)
        
        # add subscriber for action completion
        rospy.Subscriber("/execution_done", JointState, self.execution_done_callback)

    def setup_allegro_hand(self):
        # initialize joint state, set joint names and initial values
        self.joint_state = JointState()
        self.joint_state.name = [
            'joint_0.0', 'joint_1.0', 'joint_2.0', 'joint_3.0',  # index finger
            'joint_4.0', 'joint_5.0', 'joint_6.0', 'joint_7.0',  # middle finger
            'joint_8.0', 'joint_9.0', 'joint_10.0', 'joint_11.0',  # ring finger
            'joint_12.0', 'joint_13.0', 'joint_14.0', 'joint_15.0'  # thumb
        ]
        self.joint_state.position = [0.0] * 16


    def validate_pose(self, pose):
        """validate if the target pose is within the workspace"""
        # position limits
        pos_limits = {
            'x': (-1, 1),
            'y': (-1, 1),
            'z': (0.03, 1.0)
        }
        
        if not (pos_limits['x'][0] <= pose.position.x <= pos_limits['x'][1] and
                pos_limits['y'][0] <= pose.position.y <= pos_limits['y'][1] and
                pos_limits['z'][0] <= pose.position.z <= pos_limits['z'][1]):
            rospy.logwarn(f"target position out of limits: [{pose.position.x}, {pose.position.y}, {pose.position.z}]")
            return False
        
        # validate quaternion normalization
        quat_norm = np.sqrt(pose.orientation.x**2 + pose.orientation.y**2 + 
                           pose.orientation.z**2 + pose.orientation.w**2)
        if not 0.99 <= quat_norm <= 1.01:
            rospy.logwarn(f"quaternion not normalized: {quat_norm}")
            return False
        
        return True

    def execution_done_callback(self, msg):
        """handle action execution completion"""
        self.action_completed.set()
        rospy.loginfo("action execution completed")

    def action_callback(self, msg):
        with self.execution_lock:
            if self.executing:
                rospy.logwarn("previous action is still executing, ignore new action")
                return
            self.executing = True
            # clear the previous completion flag
            self.action_completed.clear()
        
        try:
            # get current pose
            current_pose = self.arm_group.get_current_pose().pose
            
            # set target pose
            target_pose = Pose()
            target_pose.position.x = msg.position[0]
            target_pose.position.y = msg.position[1]
            target_pose.position.z = msg.position[2]
            
            # use current pose
            # target_pose.orientation = current_pose.orientation
            # set target quaternion
            target_orientation = np.array(msg.position[3:7])  # get quaternion x, y, z, w
            quat_norm = np.linalg.norm(target_orientation)
            if quat_norm < 1e-6:
                rospy.logwarn("received quaternion is close to zero vector, ignore the action")
                return

            # normalize quaternion
            target_orientation /= quat_norm
            target_pose.orientation.x = target_orientation[0]
            target_pose.orientation.y = target_orientation[1]
            target_pose.orientation.z = target_orientation[2]
            target_pose.orientation.w = target_orientation[3]
            # print position difference
            dx = abs(target_pose.position.x - current_pose.position.x)
            dy = abs(target_pose.position.y - current_pose.position.y)
            dz = abs(target_pose.position.z - current_pose.position.z)
            rospy.loginfo(f"Position diff: dx={dx:.4f}, dy={dy:.4f}, dz={dz:.4f}")
            
            # if position change is too large, limit the step
            max_step = 1 # 每次最大移动20cm
            if dx > max_step or dy > max_step or dz > max_step:
                scale = max_step / max(dx, dy, dz)
                target_pose.position.x = current_pose.position.x + (target_pose.position.x - current_pose.position.x) * scale
                target_pose.position.y = current_pose.position.y + (target_pose.position.y - current_pose.position.y) * scale
                target_pose.position.z = current_pose.position.z + (target_pose.position.z - current_pose.position.z) * scale
                rospy.logwarn("position change too large, limited step")
            
            # validate target pose
            if not self.validate_pose(target_pose):
                return
            
            # set target pose and plan
            self.arm_group.set_pose_target(target_pose)
            plan = self.arm_group.plan()
            
            if isinstance(plan, tuple):
                plan_success = plan[0]
                trajectory = plan[1]
            else:
                plan_success = plan
                trajectory = plan
            
            if not plan_success:
                rospy.logwarn("motion planning failed")
                return
            
            # execute trajectory
            rospy.loginfo("start executing trajectory...")
            execution_success = self.arm_group.execute(trajectory, wait=True)
            
            if execution_success:
                rospy.loginfo("trajectory execution successful")
                # execute Allegro hand action
                allegro_positions = msg.position[7:23]
                # if self.check_joint_limits(allegro_positions):
                self.joint_state.header.stamp = rospy.Time.now()
                self.joint_state.position = allegro_positions
                self.hand_pub.publish(self.joint_state)
                    
                # wait for hand action completion
                rospy.sleep(0.1)
                    
                # publish execution completion signal
                self.execution_done_pub.publish(msg)
                    
            else:
                rospy.logwarn("trajectory execution failed")
                
        except Exception as e:
            rospy.logerr(f"error occurred during action execution: {str(e)}")
        finally:
            with self.execution_lock:
                self.executing = False

if __name__ == '__main__':
    try:
        controller = RobotController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass