#!/usr/bin/env python

# --- imports ---
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

METER_TO_PIXEL_RATIO = 10
MAX_RANGE = 8
MIN_RANGE = 0.15
COORDINATE_OFFSET = MAX_RANGE * METER_TO_PIXEL_RATIO
GRID_WIDTH = 2 * COORDINATE_OFFSET
GRID_HEIGHT = 2 * COORDINATE_OFFSET

# --- definitions ---
def resetGrid():
    global occupancy_grid
    
    # set all values to "FREE"

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

    resetGrid()
    
    for i in range(0, len(scan_msg.ranges)):
        if(scan_msg.ranges[i] == np.inf):
            x = COORDINATE_OFFSET + int(round(MAX_RANGE * np.cos(i) * METER_TO_PIXEL_RATIO))
            y = COORDINATE_OFFSET + int(round(MAX_RANGE * np.sin(i) * METER_TO_PIXEL_RATIO))
        elif(scan_msg.ranges[i] == -np.inf):
            x = COORDINATE_OFFSET + int(round(MIN_RANGE * np.cos(i) * METER_TO_PIXEL_RATIO))
            y = COORDINATE_OFFSET + int(round(MIN_RANGE * np.sin(i) * METER_TO_PIXEL_RATIO))
        else:
            x = COORDINATE_OFFSET + int(round(scan_msg.ranges[i] * np.cos(i) * METER_TO_PIXEL_RATIO))
            y = COORDINATE_OFFSET + int(round(scan_msg.ranges[i] * np.sin(i) * METER_TO_PIXEL_RATIO))
            
        if(y != 0):
            occupancy_grid.data[(y - 1) * occupancy_grid.info.width + x] = scan_msg.intensities[i]
        else:
            occupancy_grid.data[x] = scan_msg.intensities[i]
    
    pub_grid.publish(occupancy_grid)


# --- main ---
rospy.init_node("scan_grid")

# init occupancy grid
occupancy_grid = OccupancyGrid()
occupancy_grid.header.frame_id = "laser"
occupancy_grid.info.resolution = 1 # in m/cell

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
