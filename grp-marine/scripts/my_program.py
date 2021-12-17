#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

command = 0
past_command = 0
timer = 0

def move(data):
    global command
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    move_cmd = Twist()
    if command == -1 :
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = -0.3
    elif command == 1 :
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.3
    else :
        move_cmd.linear.x = 0.3
        move_cmd.angular.z = 0.0
    rospy.loginfo(command)
    pub.publish(move_cmd)

def decision(obstacles):
    global command
    global past_command
    moy = 0
    nb = len(obstacles)
    for obstacle in obstacles :
        if 0.0 < obstacle[0] and obstacle[0] < 1.0 :
            if -0.5 < obstacle[1] and obstacle[1] < 0.5 :
                moy += obstacle[1]
    moy = moy / nb
    if moy >= 0 :
        command = -1
    else :
        command = 1

    if past_command != 0 and command != past_command :
        command = past_command

    past_command = command

def detect_collision(obstacles):
    global command
    global past_command
    global timer
    for obstacle in obstacles : 
        if 0.0 < obstacle[0] and obstacle[0] < 0.6 :
            if -0.25 < obstacle[1] and obstacle[1] < 0.25 :
                if command == 0 and timer == 0 :
                    timer = 5
                    decision(obstacles)
                rospy.loginfo("obstacle détecté")
            else : 
                if timer == 0 :
                    command = 0
                    past_command = command
        else :
            if timer == 0 :
                command = 0
                past_command = command
    if timer != 0 :
        timer -= 1

def detect_obstacles(data):
    obstacles = []
    angle= data.angle_min
    for aDistance in data.ranges :
        if 0.1 < aDistance and aDistance < 1.0 :
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
    rospy.Subscriber("base_scan", LaserScan, detect_obstacles)
    rospy.Timer(rospy.Duration(0.1), move)

    rospy.spin()

if __name__ == '__main__':
    main_prog()