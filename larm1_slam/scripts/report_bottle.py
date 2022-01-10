#!/usr/bin/env python3
from os import stat
import rospy
from random import Random, randint
from rospy import rostime
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
import tf

# Initialize a global variable
# tfListener = 0
pub = 0
count = 1

def locateInMap(data):
    # bottle = PoseStamped()
    # bottle.header.stamp = rospy.Time.now()
    # bottle.header.frame_id = "odom"
    # bottle.pose.position.x = 1.0

    # global tfListener
    # bottleInMap = PoseStamped()
    # bottleInMap = tfListener.transformPose("map",bottle)

    marker = Marker()

    marker.header.frame_id = "base_footprint"
    marker.header.stamp = rospy.Time.now()

    global count
    marker.id = count
    count += 1

    marker.pose.position.x = randint(1,10)
    marker.pose.position.y = randint(1,10)
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    marker.type = 3

    marker.action = 0

    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.2

    marker.lifetime = rospy.Time(0)

    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0


    print(marker)
    pub.publish(marker)
    #rospy.loginfo("Merker sent")

def main_prog():
    rospy.init_node('Bottle_Seeker', anonymous=True)

    # global tfListener
    # tfListener = tf.TransformListener()
    global pub
    pub = rospy.Publisher('bottle', Marker, queue_size=10)

    rospy.Subscriber('trigger', String, locateInMap)

    rospy.spin()

if __name__ == '__main__':
    main_prog()