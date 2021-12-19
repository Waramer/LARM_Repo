#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

command = 0
timer = 0

# move(data) is the function that will publish the commands to move the robot
# depending on the 'command' variable modified in the other functions 
def move(data):
    global command
    global timer
    pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
    move_cmd = Twist()
    if command == -1 :
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = -0.7
    elif command == 1 :
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.7
    else :
        move_cmd.linear.x = 0.2
        move_cmd.angular.z = 0.0
    rospy.loginfo(str(command) + " : " + str(timer))
    pub.publish(move_cmd)
    if timer != 0 :
        timer -= 1

# decision(obstacles) is called when a collision is detected and will 
# analyse the environnement to choose in with direction it is better to
# turn to avoid the collision
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

# detect_collision(obstacles) will decide if there is a collision danger 
# with one of the obstacles detected and will then execute decision
def detect_collision(obstacles):
    global command
    global timer
    if timer == 0 : 
        command = 0
        for obstacle in obstacles : 
            if 0.1 < obstacle[0] and obstacle[0] < 0.6 :
                if -0.3 < obstacle[1] and obstacle[1] < 0.3 :
                    decision(obstacles)


# detect_obstacles(data) uses the data provided by the topic from the 
# laser to create a list of points that are obstacles seen by the laser

def detect_obstacles(data):
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

# main_prog() contains the main structure of the program : 
# callbacks from laser scans and a regular publishing to move the robot
def main_prog():
    rospy.init_node('my_program', anonymous=True)
    rospy.Subscriber("scan", LaserScan, detect_obstacles)
    rospy.Timer(rospy.Duration(0.1), move)

    rospy.spin()

# THe program starts its main here
if __name__ == '__main__':
    main_prog()