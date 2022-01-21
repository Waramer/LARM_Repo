#!/usr/bin/env python3
from tracemalloc import start
import rospy
import numpy as np
import math
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import tf

bridge = CvBridge()
cv_depth = []
cv_color = []
pub = 0
detect = 0
tfListener = 0
startAnalyse = 0

def rs_depth(data):
    """Fonction qui met à jour la variable contenant l'image de profondeur.  
    
    La fontion prend en entrée :
    - 'data' : topic /camera/aligned_depth_to_color/image_raw"""
    global cv_depth
    cv_depth = np.array(bridge.imgmsg_to_cv2(data,desired_encoding="passthrough"))
    global startAnalyse
    if startAnalyse == 0:
        startAnalyse = 1

def rs_color(data):
    """Fonction qui met à jour la variable contenant l'image en couleurs.  

    La fontion prend en entrée :
    - 'data' : topic /camera/color/image_raw"""
    global cv_color
    cv_color = np.array(bridge.imgmsg_to_cv2(data,'bgr8'))
    global startAnalyse
    if startAnalyse == 1:
        startAnalyse = 2

def createBottlePose(x,y,z):
    """Fonction qui crée un object 'PoseStamped' pour une boutielle.  

    La fontion prend en entrée :
    - 'x' : coordonnée x 
    - 'y' : coordonnée y 
    - 'z' : coordonnée z """
    pose = PoseStamped()
    pose.header.frame_id = "base_footprint"
    pose.header.stamp = rospy.Time()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 1.0
    return pose

def detect_bottle(data):
    """Fonction qui analyse l'image couleur pour trouver les bouteilles oranges par seuillage HSV. EN détermine la profondeur et envoie la donnée ainsi qu'un aperçu viseul de la détection dans le topic /detect_orange"""
    if startAnalyse != 2:
        return False

    inst_depth = cv_depth
    inst_color = cv_color

    hsv=cv2.cvtColor(inst_color,cv2.COLOR_BGR2HSV)
    color=15
    lo=np.array([color-3,220,50])
    hi=np.array([color+3,255,255])
    mask=cv2.inRange(hsv,lo,hi)
    mask=cv2.erode(mask, None, iterations=5)
    mask=cv2.dilate(mask, None, iterations=20)
    
    elements = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    for elt in elements:
        ((x,y),radius) = cv2.minEnclosingCircle(elt)
        cv2.circle(inst_color,(int(x),int(y)),int(radius),(255,0,0),4)

        depth = inst_depth[int(y)][int(x)] + 150
        width = np.shape(inst_color)[0]
        angle = (math.radians(39.5)/width)*(width/2 + (width/2-x))
        if abs(angle)<15 and depth<1500 and depth > 200 :
            x = depth/1000*math.cos(angle)
            y = depth/1000*math.sin(angle)
            bottle = createBottlePose(x,y,0.1)
            # tfListener.waitForTransform("/base_footprint","/map",rospy.Time(),rospy.Duration(0.05))
            bottleInMap = tfListener.transformPose("map",bottle)
            pub.publish(bottleInMap)
            
    detect.publish(bridge.cv2_to_imgmsg(inst_color,"passthrough"))

def main_prog():
    rospy.init_node('CameraObserver', anonymous=True)

    global tfListener
    tfListener = tf.TransformListener()

    rospy.Subscriber("camera/aligned_depth_to_color/image_raw", Image, rs_depth)
    rospy.Subscriber("camera/color/image_raw",Image,rs_color)
    rospy.Timer(rospy.Duration(0.3), detect_bottle)

    global pub
    pub = rospy.Publisher('bottle_orange', PoseStamped, queue_size=10)
    global detect
    detect = rospy.Publisher('detect_orange', Image, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    main_prog()