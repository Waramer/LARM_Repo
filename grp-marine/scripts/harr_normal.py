import rospy
import cv2
import numpy as np
import time 
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped





object_cascade=cv2.CascadeClassifier("../src/classifier/cascade.xml")  
bridge = CvBridge()
def rs_color(data):
    
    
    v_incr = 0
    v_scale = 1.4
    v_neigh = 14
   
        
    frame = np.array(bridge.imgmsg_to_cv2(data,'bgr8'))
        
        
    image=cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    object=object_cascade.detectMultiScale(image, scaleFactor=v_scale  , minNeighbors=v_neigh)
    # affichage de l'image en cours
    
    for x, y, w, h in object:
    
        print(x,y)
        
        cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
    
    cv2.imshow('Camera', frame)
    cv2.waitKey(1)
  
cv2.destroyAllWindows()
        
def listener():
    rospy.init_node('harr_normal.py')
    rospy.Subscriber("/camera/color/image_raw", Image, rs_color)
    rospy.spin()


if __name__ == '__main__':
    listener()








