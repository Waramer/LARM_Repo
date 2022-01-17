#!/usr/bin/env python3
from threading import Timer
import rospy
import math
from rospy.core import loginfo
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

move_command = Twist()
move_command.linear.x = 0.0
move_command.angular.z = 0.0

def observe(data):
    obstacles = []
    angle= data.angle_min
    for aDistance in data.ranges :
        if 0.1 < aDistance and aDistance < 1.0 :
            aPoint= [ 
                math.cos(angle) * aDistance,    # X
                math.sin(angle) * aDistance,    # Y
                aDistance,                      # rho
                angle                           # theta
            ]
            obstacles.append(aPoint)
        angle+= data.angle_increment
    avoid_collision(obstacles)

def move(vel):
    global move_command
    move_command.linear.x = vel

def stop():
    global move_command
    move_command.linear.x = 0.0

def turn(vel):
    global move_command
    move_command.angular.z = vel

def zone_coll(point,lat,lon):
    if -lat<point[1] and point[1]<lat :
        if point[0] >0 and point[2]<lon :
            return True
    return False

def analyse(obstacles):
    lat_sit = 0
    min_rng_prox = 1.0
    min_rng_coll = 1.0
    nb = 0
    nb_2 = 0

    for obstacle in obstacles:
        
        nb += 1
        if zone_coll(obstacle,0.3,1.0):  # si obstacles dans le couloir
            if obstacle[2] < min_rng_coll :
                min_rng_coll = obstacle[2]
        if zone_coll(obstacle,0.5,0.5): # si obstacle en entourage proche
            if obstacle[2] < min_rng_prox:
                min_rng_prox = obstacle[2]
        if zone_coll(obstacle,0.4,1.0) :
            nb_2 += 1
            if obstacle[1]>0 :
                lat_sit -= (1-obstacle[2])*(1-obstacle[1])
            else :
                lat_sit -= (1-obstacle[2])*(obstacle[1]-1)
       
    if nb_2 != 0 :
        lat_sit /= nb_2
    
    return lat_sit,min_rng_prox,min_rng_coll

def avoid_collision(obstacles):
    lat_sit,min_r_p,min_r_c= analyse(obstacles)

    if math.fabs(lat_sit) > 1.0 :
        turn(lat_sit/math.fabs(lat_sit))
    elif math.fabs(lat_sit) > 0.01 :
        turn(2*lat_sit)
    else :
        turn(0)
    if min_r_c < 0.3:
        stop()
    else:
        move(0.3*min_r_c)

    transmit()

def transmit():
    global move_command
    pub = rospy.Publisher('cas_cmd', Twist, queue_size=10)
    pub.publish(move_command)

def main_prog():
    rospy.init_node('Driver', anonymous=True)
    rospy.Subscriber("base_scan", LaserScan, observe)
    rospy.spin()

if __name__ == '__main__':
    main_prog()