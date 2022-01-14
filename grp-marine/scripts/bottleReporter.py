#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
import numpy as np
import rospkg
import sys

def get_pkg_path():
    rospack = rospkg.RosPack()
    return rospack.get_path('grp-marine')
my_pkg = get_pkg_path()
sys.path.append(my_pkg)
from scripts import bottle as bt

# global variables
pub = 0
bottles = []

def createMarker(pos,color,id):
    marker = Marker()
    marker.id = id
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.pose.position.x = pos[0]
    marker.pose.position.y = pos[1]
    marker.pose.position.z = pos[2]
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
    if color == "orange":
        marker.color.r = 2.0
        marker.color.g = 0.7
        marker.color.b = 0.0
    else :
        marker.color.r = 0.1
        marker.color.g = 0.1
        marker.color.b = 0.1
    marker.color.a = 1.0

    pub.publish(marker)

def printBottles():
    if len(bottles)>0:
        print("")
        for bottle in bottles:
            print(bottle)

def updateMap(data):
    for bottle in bottles:
        createMarker(bottle.pos,bottle.color,bottle.id)
    printBottles()

def updateOrangeBottles(data):
    global bottles
    uncertainBottlePos = [data.pose.position.x,data.pose.position.y,0.1]
    for bottle in bottles:
        if bottle.sameBottle(uncertainBottlePos,"orange"):
            bottle.addReportAndUpdate(uncertainBottlePos)
            return False
    newBottle = bt.Bottle(uncertainBottlePos,"orange")
    bottles.append(newBottle)

def main_prog():
    global bottles
    bottles = []
    rospy.init_node('Bottle_Reporter', anonymous=True)
    global pub
    pub = rospy.Publisher('bottle', Marker, queue_size=10)
    rospy.Subscriber('bottle_orange',PoseStamped,updateOrangeBottles)
    rospy.Timer(rospy.Duration(0.05), updateMap)
    rospy.spin()

if __name__ == '__main__':
    main_prog()