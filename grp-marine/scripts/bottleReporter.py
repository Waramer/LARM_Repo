#!/usr/bin/env python3
import sys
from os import stat,getcwd

from numpy.lib.function_base import append
import rospy
from random import randint
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import tf
# from bottle import Bottle

# global variables
pub = 0
marker_id = 1
bottlesTemp = []
bottles = []

def createMarker(x,y,z,color):
    marker = Marker()
    global marker_id
    marker.id = marker_id
    marker_id += 1
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.type = 3
    marker.action = 0
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.2
    marker.lifetime = rospy.Time(2.0)
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 1.0

    pub.publish(marker)

def addToTemp(data):
    bottles.append([data.pose.position.x,data.pose.position.y,data.pose.position.z,(3,1,0)])

def updateBottles(data):
    for bottle in bottles:
        reportOnMap(bottle)

def reportOnMap(bottle):
    createMarker(bottle[0],bottle[1],bottle[2],bottle[3])

def main_prog():
    rospy.init_node('Bottle_Seeker', anonymous=True)
    global pub
    pub = rospy.Publisher('bottle', Marker, queue_size=10)
    rospy.Subscriber('bottle_orange',PoseStamped,addToTemp)
    rospy.Timer(rospy.Duration(0.5),updateBottles)
    rospy.spin()

if __name__ == '__main__':
    main_prog()