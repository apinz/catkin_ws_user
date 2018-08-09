#!/usr/bin/env python

# --- imports ---
import rospy
import numpy as np
from collections import deque
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16, UInt8
import time


class Steering_calibration:

    def __init__(self, angle):
	self.start_time_millis = None
        self.first = True
	self.last = False
	self.middle = False
	self.angle = angle
	self.steer = True
	self.first_coordinates = None
	self.middle_coordinates = None
	self.last_coordinates = None
	self.sub_scan = rospy.Subscriber("/AljoschaTim/scan", LaserScan, self.scan_callback, queue_size=100)
	self.steering_pub = rospy.Publisher("AljoschaTim/steering", UInt8, queue_size=1)
        self.speed_pub = rospy.Publisher("AljoschaTim/speed", Int16, queue_size = 10, latch=True)

    #Gets an array of ranges and returns the minimum range inside that array
    def get_minimum_distance(self, ranges, range_max):
        minimum = range_max
        for x in ranges:
	    if x != float('Inf') and x < minimum:
	        minimum = x
        return minimum

    def get_maximum_distance_index(self, ranges):
        maximum = 0.0
        maximum_index = 0
        current_index = 0
        for x in ranges:
	    if x != float('inf') and x > maximum:
	        maximum = x
	        maximum_index = current_index
	    current_index += 1
        return maximum_index

    #Gets an array of ranges and returns an array of two elements, the first is the array for the x coordinates and the second is the array for the y coordinates
    def get_x_and_y_array(self, ranges):
        maximum_index = self.get_maximum_distance_index(ranges)
        x_array = ranges[:maximum_index]
        y_array = ranges[maximum_index:]
        return [x_array, y_array]	    

    #convert scan measurements into an occupancy grid
    def scan_callback(self, scan_msg):
	self.receive_two_coordinates()
        x_array, y_array = self.get_x_and_y_array(scan_msg.ranges)
        x_coordinate = self.get_minimum_distance(x_array, scan_msg.range_max)
        y_coordinate = self.get_minimum_distance(y_array, scan_msg.range_max)
        if self.first != None and self.first:
	    print("setting first coordinates")
	    self.first_coordinates = [x_coordinate, y_coordinate]
	    self.first = None
	if self.middle != None and self.middle:
	    self.middle_coordinates = [x_coordinate, y_coordinate]
	    self.middle = None
        if self.last != None and self.last:
	    self.last_coordinates = [x_coordinate, y_coordinate]
	    self.last = None
	    print([self.first_coordinates, self.middle_coordinates, self.last_coordinates])
	    exit()

    #get the current coordinates, turn the wheel to the angle, drive for one second backwards with speed -150, get the coordinates again and return both coordinates
    def receive_two_coordinates(self):
	#steer a little bit in the beginning
	if self.steer:
	    print("setting angle")
	    self.steering_pub.publish(UInt8(self.angle))
	    self.start_time_millis = int(round(time.time() * 1000))
	    self.steer = False
	
	# Wait for 2 seconds after steering, then drive backwards
	if int(round(time.time() * 1000)) > self.start_time_millis + 3000:
	    print("driving backwards")
            self.speed_pub.publish(Int16(-200))
	# Get middle coordinates
	if int(round(time.time() * 1000)) > self.start_time_millis + 4000 and self.first != None:
	    print("gettin while driveing coodinates")
	    self.middle = True
	# Wait for 1 second after driving backwards, then stop
	if int(round(time.time() * 1000)) > self.start_time_millis + 5000 and self.middle != None:
	    print("stopping")
	    self.speed_pub.publish(Int16(0))
	# Wait for 1 Second after stopping, then get the coordinates
	if int(round(time.time() * 1000)) > self.start_time_millis + 6000 and self.last != None:
	    print("set last to true")
	    self.speed_pub.publish(Int16(0))
	    self.last = True

# --- main ---
rospy.init_node("steeringCali")
Steering_calibration(45)
rospy.spin()
