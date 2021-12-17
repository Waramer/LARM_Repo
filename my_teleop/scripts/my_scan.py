#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
from std_msgs import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "  Heard : " + str(len(data.ranges)))

def transmit(data):
    pub = rospy.Publisher('coll_alert', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rospy.loginfo(data)
        pub.publish(data)
        rate.sleep()

def detect_collision(data):
    obstacles= []
    angle= data.angle_min
    for aDistance in data.ranges :
        if 0.1 < aDistance and aDistance < 2.0 :
            aPoint= [ 
                math.cos(angle) * aDistance, 
                math.sin(angle) * aDistance
            ]
            obstacles.append(aPoint)
        angle+= data.angle_increment

    for obstacle in obstacles : 
        if 0.0 < obstacle[0] and obstacle[0] < 0.5 :
            if -0.2 < obstacle[1] and obstacle[1] < 0.2 :
                try:
                    transmit("coll")
                except rospy.ROSInterruptException:
                    pass
            else :
                transmit("ras")
        else :
            transmit("ras")

#rospy.loginfo(str([ [ round(p[0], 2), round(p[1], 2) ] for p in  obstacles[0:10]]) + " ..." )

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("base_scan", LaserScan, detect_collision)
    rospy.spin()

if __name__ == '__main__':
    listener()