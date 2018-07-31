#!/usr/bin/env python

# --- imports ---
import rospy
import numpy as np
from collections import deque
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

MAX_RANGE = 8
MIN_RANGE = 0.15
GRID_RESOLUTION = 0.05
GRID_WIDTH = int(MAX_RANGE / GRID_RESOLUTION)
GRID_HEIGHT = GRID_WIDTH
BUFFER_SIZE = 1 * 360

# set all values to "FREE"
def resetGrid():
    global occupancy_grid
    occupancy_grid.data = np.zeros(occupancy_grid.info.width * occupancy_grid.info.height)

# to a given cartesian x,y coordinate, mark the corresponding cell in the grid as "OCCUPIED"
def setCell(x,y):
    global occupancy_grid
    
    res = occupancy_grid.info.resolution
    x_scaled = (x * 1.0 / res) + occupancy_grid.info.width/2
    y_scaled = (y * 1.0 / res) + occupancy_grid.info.height/2

    if x_scaled >= occupancy_grid.info.width or x_scaled < 0 or y_scaled >= occupancy_grid.info.height or y_scaled < 0:
        return

    offset = (int(round(x_scaled)) - 1) * occupancy_grid.info.height
    occupancy_grid.data[int(offset) + int(round(y_scaled) - 1)] = 100


# convert scan measurements into an occupancy grid
def scanCallback(scan_msg):
    global occupancy_grid
    global occupied_cells
    
    resetGrid()
    
    for i in range(0, len(scan_msg.ranges)):
        if(scan_msg.ranges[i] >= scan_msg.range_min and scan_msg.ranges[i] <= scan_msg.range_max):
            occupied_cells.append(np.array([scan_msg.ranges[i] * np.cos(i), scan_msg.ranges[i] * np.sin(i)]))
    
    for i in occupied_cells:
        setCell(i[0],i[1])
    
    pub_grid.publish(occupancy_grid)


# --- main ---
rospy.init_node("scan_grid")

# use deque as ringbuffer for occupied cells so we can keep a history of the laser scan measures
occupied_cells = deque(np.full((BUFFER_SIZE,2), np.inf), maxlen = BUFFER_SIZE)

# init occupancy grid
occupancy_grid = OccupancyGrid()
occupancy_grid.header.frame_id = "laser"
occupancy_grid.info.resolution = GRID_RESOLUTION # in m/cell

# width x height cells3
occupancy_grid.info.width = GRID_WIDTH
occupancy_grid.info.height = GRID_HEIGHT
occupancy_grid.data = np.zeros(occupancy_grid.info.width * occupancy_grid.info.height)

# origin is shifted at half of cell size * resolution
occupancy_grid.info.origin.position.x = int(-1.0 * occupancy_grid.info.width / 2.0) * occupancy_grid.info.resolution
occupancy_grid.info.origin.position.y = int(-1.0 * occupancy_grid.info.height / 2.0) * occupancy_grid.info.resolution
occupancy_grid.info.origin.position.z = 0
occupancy_grid.info.origin.orientation.x = 0
occupancy_grid.info.origin.orientation.y = 0
occupancy_grid.info.origin.orientation.z = 0
occupancy_grid.info.origin.orientation.w = 1

rospy.Subscriber("scan", LaserScan, scanCallback, queue_size=100)
pub_grid = rospy.Publisher("scan_grid", OccupancyGrid, queue_size=100)

rospy.spin()
