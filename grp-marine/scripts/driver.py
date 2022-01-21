#!/usr/bin/env python3
import dis
from os import stat
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

state = Twist()
move_command = Twist()
new_command =Twist()
new_command.linear.x = 0.0
new_command.angular.z = 0.0
dist_goal = 0
lin_rate = 0.05
ang_rate = 0.07
decision = False
pub = 0

def accel(vel,rate,cmd):
    """ La fonction détermine l'accélération transmise selon l'importance de la différence entre la nouvelle commande et l'ancienne.
    
        La fonction prend en entrée :
        - 'vel' : commande en cours
        - 'rate' : coefficient de la rampe 
        - ' cmd' : nouvelle commande """
    if math.fabs(cmd-vel) < 0.01 :
        return cmd
    else :
        return vel+rate*(cmd-vel)

def updatenav(data):
    global move_command
    move_command = data

def updatecas(data):
    """ La fonction met à jour les commandes et donne la priorité au CAS si la commande n'est pas standard.
    
        La fonction prend en entrée :
        - 'data' : commande du CAS"""
    global decision
    global new_command
    new_command =data
    if (new_command.linear.x != 0.3 and new_command.angular.z != 0.0) :
        decision =True
    else :
        decision = False

def updateDist(data):
    global dist_goal
    dist_goal = data.data

def move(data):
    """ La fonction donne la priorité au CAS sur la NAV en fonction de la commande reçu"""
    global state
    global move_command
    global lin_rate
    global ang_rate
    global new_command
    global decision
    if decision==True and dist_goal>0.3: 
        rospy.loginfo("CAS")
        state.linear.x = accel(state.linear.x,lin_rate,new_command.linear.x)
        state.angular.z = accel(state.angular.z,ang_rate,new_command.angular.z)
    else:
        rospy.loginfo("NAV")
        state.linear.x = accel(state.linear.x,lin_rate,move_command.linear.x)
        state.angular.z = accel(state.angular.z,ang_rate,move_command.angular.z)
        
    global pub
    pub.publish(state)



def main_prog():
    rospy.init_node('Driver', anonymous=True)
    rospy.Subscriber("nav_cmd", Twist, updatenav)
    rospy.Subscriber("cas_cmd", Twist, updatecas)
    rospy.Subscriber("dist_goal", Float32, updateDist)
    global pub
    pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
    rospy.Timer(rospy.Duration(0.05), move)
    rospy.spin()

if __name__ == '__main__':
    main_prog()