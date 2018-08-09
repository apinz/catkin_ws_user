#!/usr/bin/env python2
import rospy
import roslib
import sys
import math
import numpy as np
import cv2
import os

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from cv_bridge import CvBridge, CvBridgeError

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

DEPTH_THRESHOLD = 900
EDGE_KERNEL = np.array([[1.0,1.0,1.0], [1.0,-8.0,1.0], [1.0,1.0,1.0]])

# sizes are in cm
MAP_SIZE_X = 600
MAP_SIZE_Y = 400
MAP_RESOLUTION = 10

class CameraLocalization:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_image_sub = rospy.Subscriber("/AljoschaTim/app/camera/depth/image_raw", Image, self.depthImageCallback, queue_size = 1)
        self.odometry_sub = rospy.Subscriber("/localization/odom/1", Odometry, self.odometry_callback, queue_size = 1)

    def odometry_callback(self, data):
        return

    def depthImageCallback(self, depth_data):
        try:
            cv_depth_image = self.bridge.imgmsg_to_cv2(depth_data, "16UC1")
        except CvBridgeError as e:
            print(e)
         
        binary_array = np.empty(cv_depth_image.shape, dtype=np.uint8) 
        for x in range(cv_depth_image.shape[0]):
            for y in range(cv_depth_image.shape[1]):
                if cv_depth_image[x][y] < DEPTH_THRESHOLD and cv_depth_image[x][y] > 0:
                    binary_array[x][y] = 255
                else:
                    binary_array[x][y] = 0                   
        
        # get lane edges
        edges = cv2.filter2D(binary_array, -1, EDGE_KERNEL)
        cv2.imwrite('./edges.png', edges)

def main():
    print("Running\n")
    rospy.init_node('camera_localization')    
    camera_localization = CameraLocalization()
    rospy.spin()

if __name__ == '__main__':
    main()
