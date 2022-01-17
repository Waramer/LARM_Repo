#!/usr/bin/env python3
from os import stat
import rospy
import math
from geometry_msgs.msg import Twist

state = Twist()
move_command = Twist()
lin_rate = 0.05
ang_rate = 0.07

def accel(vel,rate,cmd):
    if math.fabs(cmd-vel) < 0.01 :
        return cmd
    else :
        return vel+rate*(cmd-vel)

def update(data):
    global move_command
    move_command = data

def move(data):
    global state
    global move_command
    global lin_rate
    global ang_rate

    state.linear.x = accel(state.linear.x,lin_rate,move_command.linear.x)
    state.angular.z = accel(state.angular.z,ang_rate,move_command.angular.z)

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    pub.publish(state)

def main_prog():
    rospy.init_node('Driver', anonymous=True)
    rospy.Subscriber("nav_cmd", Twist, update)
    rospy.Subscriber("cas_cmd", Twist, update)
    rospy.Timer(rospy.Duration(0.05), move)
    rospy.spin()

if __name__ == '__main__':
    main_prog()