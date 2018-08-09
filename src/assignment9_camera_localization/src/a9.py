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
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

DEPTH_THRESHOLD = 900

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

def getLoopCoords(self, points):
    # map the points to the lower resolution, no duplicates
    x_index = np.int(x * self.resolution)
    y_index = np.int(y * self.resolution)
    
    if (x_index < 0):
        x_index = 0
    if (x_index > ((self.map_size_x / self.resolution) - 1)):
        x_index = (self.map_size_x / self.resolution) - 1

    if (y_index < 0):
        y_index = 0
    if (y_index > ((self.map_size_y / self.resolution) - 1)):
        y_index = (self.map_size_y / self.resolution) - 1

    x_cell, y_cell = self.matrix[x_index, y_index, :]

def getDistanceMap(self):
    # calculate the smallest distance for each of the 2400 points in the map to the loop coords
    for x in range(int(MAP_SIZE_X / MAP_RESOLUTION)):
        for y in range(int(MAP_SIZE_Y / MAP_RESOLUTION)):
            point_diff = p - q
            sqrt(point_diff.x * point_diff.x + point_diff.y * point_diff.y)
    
def readPoints(map_file, offset_x=0.0, offset_y=0.0):
    """
    Reads a file with the map data in the RNDF Format
    :return: generator of x, y position tuples
    """
    # detect waypoints x.y.z and store latitude / longitude as x / y
    with open(map_file) as m_file:
        for line in m_file:
            if line.startswith('1.'):
                x, y = line.split('\t')[1:3]
                yield float(x) + offset_x, float(y) + offset_y
    with open(map_file) as m_file:
        for line in m_file:
            if line.startswith('1.'):
                x, y = line.split('\t')[1:3]
                yield float(x) + offset_x, float(y) + offset_y

def readMap(map_file):
    """
    Read the file with the map data and return a Trajectory.
    :return: List of Pose elements
    """
    points = []

    for x, y in readPoints(map_file):
        xy_point = Point(x, y, 0)
        xy_pose  = Pose(position=xy_point)
        points.append(xy_pose)

    return points

def main():
    print("Running\n")
    rospy.init_node('camera_localization')
    
    # calculate force map for localization
    # TODO: store it so we just need to load it next time
    inner_points_file = readMap('new_map_inner_loop.txt')
    middle_points_file = readMap('new_map_inner_loop.txt')
    outer_points_file = readMap('new_map_outer_loop.txt')
    
    inner_points = getLoopCoords(inner_points_file)
    middle_points = getLoopCoords(middle_points_file)
    outer_points = getLoopCoords(outer_points_file)
    
    #~ del inner_points_file
    #~ del middle_points_file
    #~ del outer_points_file
    
    # TODO: getDistanceMap()
    
    camera_localization = CameraLocalization()
    rospy.spin()

if __name__ == '__main__':
    main()
