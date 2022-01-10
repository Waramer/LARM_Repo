#!/usr/bin/env python3
from os import stat
import rospy
from geometry_msgs.msg import PoseStamped
import tf

# Initialize a global variable
tfListener = 0

def callback(data):
    rospy.loginfo("/GOAL : "+str(data))
    global tfListener
    dataTransformed = tfListener.transformPose("base_footprint",data)
    rospy.loginfo("/BASE : "+str(dataTransformed))

def main_prog():
    rospy.init_node('Driver', anonymous=True)
    global tfListener
    tfListener = tf.TransformListener()
    rospy.Subscriber("goal", PoseStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    main_prog()