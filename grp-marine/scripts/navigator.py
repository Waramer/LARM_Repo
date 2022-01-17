#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
import tf
import numpy as np

pos = [0,0]
hdg = 0
goal = [0,0]
map = []
nav_cmd = Twist()
ang_vel = 0.7
lin_vel = 0.4
dist_secu = 1.0
pub = 0

def quaternion_to_euler(x, y, z, w):
    ysqr = y * y
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = np.degrees(np.arctan2(t0, t1))
    t2 = +2.0 * (w * y - z * x)
    t2 = np.where(t2>+1.0,+1.0,t2)
    t2 = np.where(t2<-1.0, -1.0, t2)
    Y = np.degrees(np.arcsin(t2))
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = np.degrees(np.arctan2(t3, t4))
    return X, Y, Z 

def updateMap(data):
    pass

def updatePos(data):
    global pos
    pos[0]=data.pose.pose.position.x
    pos[1]=data.pose.pose.position.y
    global hdg
    hdg = quaternion_to_euler(data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)[2]
    if hdg<0:
        hdg += 360

def updateGoal(data):
    global goal
    goal[0]=data.pose.position.x
    goal[1]=data.pose.position.y

def sendNavCommand(data):
    toGoal,goal_hdg,goal_rng = [goal[0]-pos[0],goal[1]-pos[1]],hdg,0

    if toGoal[0]!=0 and toGoal[1]!=0:
        goal_rng = math.dist([0,0],toGoal)
        goal_hdg = math.atan(toGoal[1]/toGoal[0]) * 180 / math.pi
        if toGoal[0]>0:
            if toGoal[1]<0:
                goal_hdg += 360
        else:
            goal_hdg += 180
    
    global nav_cmd
    # Angular command
    dhdg = (360+goal_hdg-hdg)%360
    if dhdg < 180:
        nav_cmd.angular.z = dhdg/180*ang_vel
    else:
        nav_cmd.angular.z = -(360-dhdg)/180*ang_vel
    # Linear command
    if goal_rng > 0.1 and (dhdg < 15 or dhdg > 345):
        nav_cmd.linear.x = np.min([goal_rng,dist_secu])/dist_secu*lin_vel
    else:
        nav_cmd.linear.x = 0

    pub.publish(nav_cmd)
    rospy.loginfo("Heading:%3d -- DHDG = %3d"%(hdg,dhdg))

def main_prog():
    rospy.init_node('Driver', anonymous=True)

    rospy.Subscriber("map", OccupancyGrid, updateMap)
    rospy.Subscriber("odom", Odometry,updatePos)
    rospy.Subscriber("goal",PoseStamped,updateGoal)

    global pub
    pub = rospy.Publisher("nav_cmd",Twist,queue_size=10)
    rospy.Timer(rospy.Duration(0.1), sendNavCommand)
    rospy.spin()

if __name__ == '__main__':
    main_prog()