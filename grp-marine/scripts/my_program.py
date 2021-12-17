#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

command = 0
timer = 0
obstacles= []

def move(data):
    global command
    global timer
    pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
    move_cmd = Twist()
    if command == -1 :
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = -0.5
    elif command == 1 :
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.5
    else :
        move_cmd.linear.x = 0.1
        move_cmd.angular.z = 0.0
    rospy.loginfo(str(command) + " : " + str(timer))
    pub.publish(move_cmd)
    if timer != 0 :
        timer -= 1

def decision(obstacles):
    global command
    global timer
    dec = 0
    nb = len(obstacles)
    for obstacle in obstacles :
        if 0.0 < obstacle[0] and obstacle[0] < 0.5 :
            if -0.2 < obstacle[1] and obstacle[1] < 0.2 :
                dec += obstacle[1]
    rospy.loginfo(str(dec))
    if dec >= 0 :
        command = -1
    else :
        command = 1
    timer = 30

def detect_collision(obstacles):
    global command
    global timer

    if timer == 0 : 
        command = 0
        for obstacle in obstacles : 
            if 0.1 < obstacle[0] and obstacle[0] < 0.6 :
                if -0.3 < obstacle[1] and obstacle[1] < 0.3 :
                    decision(obstacles)

def detect_obstacles(data):
    global obstacles
    obstacles = []
    angle= data.angle_min
    for aDistance in data.ranges :
        if 0.1 < aDistance and aDistance < 0.5 :
            aPoint= [ 
                math.cos(angle) * aDistance, 
                math.sin(angle) * aDistance
            ]
            obstacles.append(aPoint)
        angle+= data.angle_increment
    detect_collision(obstacles)

#rospy.loginfo(str([ [ round(p[0], 2), round(p[1], 2) ] for p in  obstacles[0:10]]) + " ..." )

def main_prog():
    rospy.init_node('my_program', anonymous=True)
    rospy.Subscriber("scan", LaserScan, detect_obstacles)
    rospy.Timer(rospy.Duration(0.1), move)

    rospy.spin()

if __name__ == '__main__':
    main_prog()