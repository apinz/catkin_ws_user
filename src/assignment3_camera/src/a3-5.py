#!/usr/bin/env python
import numpy as np 
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# matrix for the intrinsic parameters
fx = 614.1699
fy = 614.9002
cx = 329.9491
cy = 237.2788
intrinsic = np.matrix([[fx,0,cx],[0,fy,cy],[0,0,1]])

# vector for the distortion parameters
k1 = 0.1115
k2 = -0.1089
t1 = 0
t2 = 0
distortion = np.array([k1,k2,t1,t2])

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/image_processing/bin_img", Image, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/app/camera/rgb/image_raw", Image, self.callback, queue_size=1)


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    #make it gray
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    
    dot_color = 255
    threshold = 248
    ret, thresh_img = cv2.threshold(gray, threshold, dot_color, cv2.THRESH_BINARY)
    
    dots_x_pos = []
    dots_y_pos = []
    for x in range(thresh_img.shape[0]):
        for y in range(thresh_img.shape[1]):
            if(thresh_img[x][y] == dot_color):
                dots_x_pos.append(x)
                dots_y_pos.append(y)
    dots_xy = zip(dots_x_pos , dots_y_pos)

    cv2.imshow("Image window", thresh_img)
    cv2.imwrite('bw-dots.png', thresh_img)
    cv2.waitKey(3)
    
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(gray, "mono8"))
    except CvBridgeError as e:
      print(e) 

def main(args):
    print("Running\n")
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
