#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
from moveit_commander import PlanningSceneInterface, RobotCommander
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlanningScene, ObjectColor
from std_msgs.msg import ColorRGBA

class SceneManager(object):
    def __init__(self):
        rospy.init_node('scene_manager_node', anonymous=True)
        
        self.scene = PlanningSceneInterface()
        self.robot = RobotCommander()
        
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)
        self.colors = dict()
        
        # wait for scene interface to be initialized
        rospy.sleep(1)
        
        # clear existing scene objects
        self.clear_scene()

    def clear_scene(self):
        """clear all objects in the scene"""
        for obj in self.scene.get_known_object_names():
            self.scene.remove_world_object(obj)
        rospy.sleep(0.5)

    def set_color(self, name, r, g, b, a=1.0):
        """set object color"""
        color = ObjectColor()
        color.id = name
        color.color = ColorRGBA(r, g, b, a)
        self.colors[name] = color

    def send_colors(self):
        """send color information to the scene"""
        p = PlanningScene()
        p.is_diff = True
        for color in self.colors.values():
            p.object_colors.append(color)
        self.scene_pub.publish(p)

    def add_box_object(self, name, size, position, orientation=[0,0,0,1], frame_id=None):
        """add a box object to the scene"""
        if frame_id is None:
            frame_id = self.robot.get_planning_frame()
            
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]
        pose.pose.orientation.x = orientation[0]
        pose.pose.orientation.y = orientation[1]
        pose.pose.orientation.z = orientation[2]
        pose.pose.orientation.w = orientation[3]
        
        self.scene.add_box(name, pose, size)

    def setup_scene(self):
        """set up the whole scene"""
        try:
            # add table
            table_size = [1.8, 0.8, 0.05]  # length, width, height (meter)
            table_position = [0.35, -0.10, -0.026]  # half of the table thickness plus 0.001
            self.add_box_object('table', table_size, table_position)
            self.set_color('table', 0.8, 0.8, 0.8, 1.0)  # light gray (metal)

            # add left wall
            wall_size = [1.8, 0.05, 0.8]  # length, width, height (meter)
            wall_position = [0.35, -0.5, 0.4]  # on the left side of the table
            self.add_box_object('wall', wall_size, wall_position)
            self.set_color('wall', 0.8, 0.8, 0.8, 1.0)  # light gray (metal)

            # add other walls, obstacles, etc.

            # send color information
            self.send_colors()
            
            # wait for scene update
            rospy.sleep(1)
            
            # verify if the scene objects are added successfully
            scene_objects = self.scene.get_known_object_names()
            rospy.loginfo(f"Objects in the scene: {scene_objects}")
            
            return True
            
        except Exception as e:
            rospy.logerr(f"Error setting scene: {str(e)}")
            return False

    def check_scene_objects(self):
        """check the objects in the scene"""
        return self.scene.get_known_object_names()

def main():
    try:
        scene_manager = SceneManager()
        
        if scene_manager.setup_scene():
            rospy.loginfo("Scene is set successfully!")
        else:
            rospy.logerr("Scene setup failed!")
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.logerr("Program interrupted")
        return
    except Exception as e:
        rospy.logerr(f"Program has error: {str(e)}")
        return

if __name__ == '__main__':
    main()