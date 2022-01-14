#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker

class Bottle(Marker):
    """Bottle class created to have a template of a bottle marker with only position and color to give"""
    
    id = 1

    def __init__(self,x,y,z,color):
        global id
        self.id = id
        id += 1
        self.header.frame_id = "base_footprint"
        self.header.stamp = rospy.Time.now()
        self.pose.position.x = x
        self.pose.position.y = y
        self.pose.position.z = z
        self.pose.orientation.x = 0.0
        self.pose.orientation.y = 0.0
        self.pose.orientation.z = 0.0
        self.pose.orientation.w = 1.0
        self.type = 3
        self.action = 0
        self.scale.x = 0.1
        self.scale.y = 0.1
        self.scale.z = 0.2
        self.lifetime = rospy.Time(0)
        self.color.r = color[0]
        self.color.g = color[1]
        self.color.b = color[2]
        self.color.a = 1.0

    def publish(self,pub):
        pub.publish(self)
        rospy.loginfo("Merker sent")
        rospy.loginfo(self)


