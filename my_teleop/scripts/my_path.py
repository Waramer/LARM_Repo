#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist

def talker():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():

        move_cmd = Twist()
        move_cmd.linear.x = 2.0
        move_cmd.angular.z = 2.0

        rospy.loginfo(move_cmd)
        pub.publish(move_cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass