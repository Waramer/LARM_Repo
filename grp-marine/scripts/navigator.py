#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import numpy as np
import cv2
import rospkg

pos = [0,0]
hdg = 0
goal = [0,0]
map = []
nav_cmd = Twist()
ang_vel = 0.7
lin_vel = 0.4
dist_secu = 2.0
pub_cmd = 0
pub_goal = 0

def get_pkg_path():
    rospack = rospkg.RosPack()
    return rospack.get_path('grp-marine')

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
    width = data.info.width
    height = data.info.height
    global map
    map = np.zeros((height,width,3))
    for k in range(0,width*height):
        i = k//width
        j = k%width
        if data.data[k] == -1:
            map[-1-i][j] = [0.3,0.3,0.3]
        elif data.data[k] == 0:
            map[-1-i][j] = [1,1,1]
        elif data.data[k] == 100:
            map[-1-i][j] = [0,0,0]

    cv2.imshow("MAP",map)
    file = get_pkg_path()+"src/scripts/map.png"
    cv2.imwrite(file,map)
    cv2.waitKey(1)

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
    
    dist_goal = Float32()
    dist_goal.data = goal_rng
    pub_goal.publish(dist_goal)

    global nav_cmd
    # Angular command
    dhdg = (360+goal_hdg-hdg)%360
    if goal_rng < 0.3:
        nav_cmd.angular.z = 0
    elif dhdg < 180:
        nav_cmd.angular.z = max(dhdg/180*ang_vel,0.1)
    else:
        nav_cmd.angular.z = -max((360-dhdg)/180*ang_vel,0.1)
    # Linear command
    if goal_rng > 0.3 and (dhdg < 15 or dhdg > 345):
        nav_cmd.linear.x = np.min([goal_rng,dist_secu])/dist_secu*lin_vel
    elif goal_rng < 0.2 :
        nav_cmd.linear.x = 0
    else :
        nav_cmd.linear.x = 0.1

    pub_cmd.publish(nav_cmd)

def main_prog():
    rospy.init_node('Driver', anonymous=True)

    rospy.Subscriber("map", OccupancyGrid, updateMap)
    rospy.Subscriber("odom", Odometry,updatePos)
    rospy.Subscriber("goal",PoseStamped,updateGoal)

    global pub_cmd
    pub_cmd = rospy.Publisher("nav_cmd",Twist,queue_size=10)
    global pub_goal
    pub_goal = rospy.Publisher("dist_goal",Float32,queue_size=10)
    rospy.Timer(rospy.Duration(0.1), sendNavCommand)
    rospy.spin()

if __name__ == '__main__':
    main_prog()