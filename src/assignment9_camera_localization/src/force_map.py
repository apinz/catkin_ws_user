#!/usr/bin/env python2
import sys
import math
import numpy as np
import cv2
import os

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt

DEPTH_THRESHOLD = 900

# sizes are in cm
MAP_SIZE_X = 600
MAP_SIZE_Y = 400
MAP_RESOLUTION = 10

def getLoopCoords(points):
    # map the points to the lower resolution, no duplicates    
    coord_set = set()
    coord_list = []
    
    for i in range(len(points)):
        x_low_res = np.int(points[i].x * MAP_RESOLUTION)
        y_low_res = np.int(points[i].y * MAP_RESOLUTION)
        xy_point = Point(x_low_res, y_low_res, 0)
        if(not xy_point in coord_set):
            coord_set.add(xy_point)
            coord_list.append(xy_point)

    return coord_list
        
def getDistanceMap(points):
    # calculate the smallest distance for each of the 2400 points in the map to the loop coords
    min_point_distances = []
    
    for x in range(int(MAP_SIZE_X / MAP_RESOLUTION)):
        for y in range(int(MAP_SIZE_Y / MAP_RESOLUTION)):
            pq_dist = 10000
            q = Point(x, y, 0)
            
            for p in points:
                point_diff = Point(float(p.x) - float(q.x), float(p.y) - float(q.y), 0)
                distance = math.sqrt(point_diff.x * point_diff.x + point_diff.y * point_diff.y)
                if(distance < pq_dist):
                    pq_dist = distance
                    pq = point_diff
                    
            min_point_distances.append(pq)
            print len(min_point_distances)
    
    return min_point_distances
                    
def readPoints(map_file, offset_x = 0.0, offset_y = 0.0):
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
        points.append(xy_point)

    return points

def main():
    print("Running\n")
    
    # calculate force map for localization
    # TODO: store it so we just need to load it next time
    inner_points_file = readMap('new_map_inner_loop.txt')
    middle_points_file = readMap('new_map_inner_loop.txt')
    outer_points_file = readMap('new_map_outer_loop.txt')
    
    inner_points = getLoopCoords(inner_points_file)
    middle_points = getLoopCoords(middle_points_file)
    outer_points = getLoopCoords(outer_points_file)
        
    min_point_distances = getDistanceMap(inner_points + middle_points + outer_points)    
    print(str(min_point_distances))

    X = []
    Y = []
    U = []
    V = []
    for x in range(int(MAP_SIZE_X / MAP_RESOLUTION)):
        for y in range(int(MAP_SIZE_Y / MAP_RESOLUTION)):
            X.append(x)
            Y.append(y)

    for p in min_point_distances:
        U.append(p.x)
        V.append(p.y)

    fig, ax = plt.subplots()
    q = ax.quiver(X, Y, U, V)
    plt.show()
    
    print("Finished")

if __name__ == '__main__':
    main()
