#!/usr/bin/env python3
from os import stat
import rospy
import math
from geometry_msgs.msg import Twist

state = Twist()
move_command = Twist()
new_command =Twist()
new_command.linear.x = 0.0
new_command.angular.z = 0.0
lin_rate = 0.05
ang_rate = 0.07
decision = False

def accel(vel,rate,cmd):
    if math.fabs(cmd-vel) < 0.01 :
        return cmd
    else :
        return vel+rate*(cmd-vel)

def updatenav(data):
    global move_command
    move_command = data

def updatecas(data):
    global decision
    global new_command
    new_command =data
    if (new_command.linear.x != 0.3 and new_command.angular.z != 0.0) :
        decision =True
    else :
        decision = False


def move(data):
    global state
    global move_command
    global lin_rate
    global ang_rate
    global new_command
    global decision
    if decision==True:
        print("lol")
        state.linear.x = accel(state.linear.x,lin_rate,new_command.linear.x)
        state.angular.z = accel(state.angular.z,ang_rate,new_command.angular.z)
    else:
        state.linear.x = accel(state.linear.x,lin_rate,move_command.linear.x)
        state.angular.z = accel(state.angular.z,ang_rate,move_command.angular.z)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    pub.publish(state)



def main_prog():
    rospy.init_node('Driver', anonymous=True)
    rospy.Subscriber("nav_cmd", Twist, updatenav)
    rospy.Subscriber("cas_cmd", Twist, updatecas)
    rospy.Timer(rospy.Duration(0.05), move)
    rospy.spin()

if __name__ == '__main__':
    main_prog()