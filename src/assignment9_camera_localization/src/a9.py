#!/usr/bin/env python2
import rospy
import roslib
import sys
import math
import numpy as np
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

PATH = "/root/catkin_ws_user/src/assignment9_camera_localization/src/"

class CameraLocalization:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_image_sub = rospy.Subscriber("/AljoschaTim/app/camera/depth/image_raw", Image, self.depthImageCallback, queue_size = 1)
        self.ir_image_sub = rospy.Subscriber("/AljoschaTim/app/camera/ir/image_raw", Image, self.irImageCallback, queue_size = 1)
    
    def depthImageCallback(self, depthData):
        #print(str(depthData))
        depthImage = self.bridge.imgmsg_to_cv2(depthData, "16UC1")
        depthArray = np.array(depthImage, dtype=np.uint16)
        cv2.imwrite("./depth_img.png", depthArray)
        
    def irImageCallback(self, irData):
        print(str(irData))
        irImage = self.bridge.imgmsg_to_cv2(irData, "16UC1")
        irArray = np.array(irImage, dtype=np.uint16)
        cv2.imwrite("./ir_img.png", irImage)

def main():
    print("Running\n")
    rospy.init_node('camera_localization')
    camera_localization = CameraLocalization()
    rospy.spin()

if __name__ == '__main__':
    main()
