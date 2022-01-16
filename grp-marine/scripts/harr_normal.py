import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import rospkg
import math
import tf

def get_pkg_path():
    rospack = rospkg.RosPack()
    return rospack.get_path('grp-marine')
my_pkg = get_pkg_path()
object_cascade=cv2.CascadeClassifier(my_pkg+"/src/classifier/cascade.xml")  
bridge = CvBridge()
cv_depth = []
cv_dolor = []
pub = 0
detect = 0
tfListener = 0

def rs_color(data):
    global cv_color
    cv_color = np.array(bridge.imgmsg_to_cv2(data,'bgr8'))

def rs_depth(data):
    global cv_depth
    cv_depth = np.array(bridge.imgmsg_to_cv2(data,desired_encoding="passthrough"))

def createBottlePose(x,y,z):
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
    
    inst_depth = cv_depth
    inst_color = cv_color

    v_incr = 0
    v_scale = 1.65
    v_neigh = 28
        
    image=cv2.cvtColor(inst_color, cv2.COLOR_BGR2GRAY)
    object=object_cascade.detectMultiScale(image, scaleFactor=v_scale  , minNeighbors=v_neigh)
    
    for x, y, w, h in object:
        xb = x+w/2
        yb = y+h/2
        cv2.rectangle(inst_color, (x, y), (x+w, y+h), (255, 0, 0), 2)

        depth = inst_depth[int(yb)][int(xb)] + 150
        width = np.shape(inst_color)[0]
        angle = (math.radians(39.5)/width)*(width/2 + (width/2-xb))
        if abs(angle)<15 and depth<1500 and depth > 200 :
            x = depth/1000*math.cos(angle)
            y = depth/1000*math.sin(angle)
            bottle = createBottlePose(x,y,0.1)
            # tfListener.waitForTransform("/base_footprint","/map",rospy.Time(),rospy.Duration(0.05))
            bottleInMap = tfListener.transformPose("map",bottle)
            pub.publish(bottleInMap)    

    detect.publish(bridge.cv2_to_imgmsg(inst_color,"passthrough"))

def main_prog():
    rospy.init_node('harr_normal.py')

    global tfListener
    tfListener = tf.TransformListener()

    rospy.Subscriber("camera/aligned_depth_to_color/image_raw", Image, rs_depth)
    rospy.Subscriber("camera/color/image_raw",Image,rs_color)
    rospy.Timer(rospy.Duration(0.3), detect_bottle)

    global pub
    pub = rospy.Publisher('bottle_black', PoseStamped, queue_size=10)
    global detect
    detect = rospy.Publisher('detect_black', Image, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    main_prog()
