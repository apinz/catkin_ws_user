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

RGB_LOWERB = np.array([238, 238, 238])
RGB_UPPERB = np.array([255, 255, 255])

HSV_LOWERB = np.array([0, 0, 235])
HSV_UPPERB = np.array([179, 20, 255])

YUV_LOWERB = np.array([248, 0, 0])
YUV_UPPERB = np.array([255, 255, 255])

class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback, queue_size=1)

  def callback(self,data):      
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # extract the lines in RGB
    mask = cv2.inRange(cv_image, RGB_LOWERB, RGB_UPPERB)
    filtered_rgb = cv2.bitwise_and(cv_image, cv_image, mask = mask)
    cv2.imwrite('./rgb_mask.png', mask)
    cv2.imwrite('./rgb_filtered.png', filtered_rgb)
    
    # extract the lines in YUV
    img_yuv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2YUV)
    mask = cv2.inRange(img_yuv, YUV_LOWERB, YUV_UPPERB)
    filtered_yuv = cv2.bitwise_and(cv_image, cv_image, mask = mask)
    cv2.imwrite('./yuv_mask.png', mask)
    cv2.imwrite('./yuv_filtered.png', filtered_yuv)
    
    # extract the lines in HSV
    hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_img, HSV_LOWERB, HSV_UPPERB)
    filtered_hsv = cv2.bitwise_and(cv_image, cv_image, mask = mask)
    cv2.imwrite('./hsv_mask.png', mask)
    cv2.imwrite('./hsv_filtered.png', filtered_hsv)

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
