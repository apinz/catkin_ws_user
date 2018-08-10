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
SCALING_FACTOR = MAP_RESOLUTION / 10

def getLoopCoords(points):
    # map the points to the lower resolution, no duplicates    
    coord_set = set()
    coord_list = []
    
    for i in range(len(points)):
        x_low_res = np.int(points[i][0] * MAP_RESOLUTION)
        y_low_res = np.int(points[i][1] * MAP_RESOLUTION)
        xy_point = Point(x_low_res, y_low_res, 0)
        if(not xy_point in coord_set):
            coord_set.add(xy_point)
            coord_list.append([x_low_res, y_low_res])

    return coord_list
        
def getDistanceMap(points):
    # calculate the smallest distance for each of the 2400 points in the map to the loop coords
    min_point_distances = []
    
    for x in range(int(MAP_SIZE_X / MAP_RESOLUTION)):
        for y in range(int(MAP_SIZE_Y / MAP_RESOLUTION)):
            pq_dist = 10000
            q = [x, y]
            
            for p in points:
                point_diff = [float(p[0]) - float(q[0]), float(p[1]) - float(q[1])]
                distance = math.sqrt(point_diff[0] * point_diff[0] + point_diff[1] * point_diff[1])
                if(distance < pq_dist):
                    pq_dist = distance
                    pq = point_diff
                    
            min_point_distances.append((pq, pq_dist))
            print(str(len(min_point_distances)) + "/" + str((MAP_SIZE_X / MAP_RESOLUTION) * (MAP_SIZE_Y / MAP_RESOLUTION)))
    
    return min_point_distances
    
def getWeightedDistances(distances_list):
    weight_dividends = []
    
    for l in distances_list:
        dividends = []
        for pqd in l:
            dividends.append(np.exp(-pqd[1] * SCALING_FACTOR))
            
        weight_dividends.append(dividends)
           
    weight_divisors = np.sum(np.array(weight_dividends), axis = 0)
    weight_list = np.divide(np.array(weight_dividends), weight_divisors)
    
    distances_vectors_list = []
    for l in distances_list:
        distance_vectors = np.array(list(zip(*l))[0])
        distances_vectors_list.append(distance_vectors)
    
    weighted_distances_list = []
    for k in range(len(distances_vectors_list)):
        weighted_distances = np.transpose(np.multiply(np.transpose(distances_vectors_list[k]), weight_list[k]))
        weighted_distances_list.append(weighted_distances)
    
    weighted_distances = np.sum(weighted_distances_list, axis = 0)
    
    return weighted_distances
                    
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
        xy_point = [x, y]
        points.append(xy_point)

    return points

def main():
    print("Running\n")
    
    # calculate force map for localization
    # TODO: store it so we just need to load it next time
    inner_points_file = readMap('new_map_inner_loop.txt')
    middle_points_file = readMap('new_map_middle_loop.txt')
    outer_points_file = readMap('new_map_outer_loop.txt')
    
    inner_points = getLoopCoords(inner_points_file)
    middle_points = getLoopCoords(middle_points_file)
    outer_points = getLoopCoords(outer_points_file)
    
    inner_points_distances = getDistanceMap(inner_points)
    middle_points_distances = getDistanceMap(middle_points)
    outer_points_distances = getDistanceMap(outer_points)
    
    #~ min_distances = getDistanceMap(inner_points + middle_points + outer_points)
    weighted_distances = getWeightedDistances([inner_points_distances, middle_points_distances, outer_points_distances])

    X = []
    Y = []
    U = []
    V = []
    for x in range(int(MAP_SIZE_X / MAP_RESOLUTION)):
        for y in range(int(MAP_SIZE_Y / MAP_RESOLUTION)):
            X.append(x)
            Y.append(y)

    for p in weighted_distances:
        U.append(p[0])
        V.append(p[1])

    fig, ax = plt.subplots()
    q = ax.quiver(X, Y, U, V)
    plt.show()
    
    print("Finished")

if __name__ == '__main__':
    main()
