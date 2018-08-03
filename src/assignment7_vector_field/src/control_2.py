#!/usr/bin/env python2
import numpy as np
import rospy
import tf
import roslib
import sys
from collections import deque

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Int16, UInt8
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose

MAX_ANGLE_RIGHT = 0
ANGLE_STRAIGHT = 90
MAX_ANGLE_LEFT = 180
SETPOINT = 3.0
INITIAL_ERROR = 0.0
YAW_TO_STEERING_FACTOR = 180.0 / np.pi
# SPEED JUNKIE SETTINGS
#~ KP = 3.6
#~ KI = 1.8
#~ KD = 0.0
#~ SPEED = 650
# KEEP CAR IN LANE WITH SPEED 520
KP = 3.6
KI = 1.8
KD = 0.50
SPEED = 650
ERROR_QUEUE_SIZE = 3
KI_UPPER_LIMIT = 15
KI_LOWER_LIMIT = -15

PATH = "/home/aljoscha/uni/informatik/master/robotics/catkin_ws_user/src/assignment7_vector_field/src/"
NAMESPACE = "/AljoschaTim"

class ForceController:
    def __init__(self):
        self.previous_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.odometry_sub = rospy.Subscriber("/localization/odom/1", Odometry, self.callback, queue_size = 1)
        self.steering_pub = rospy.Publisher("AljoschaTim/steering", UInt8, queue_size=1)
        self.pub_speed = rospy.Publisher(NAMESPACE + "/speed", Int16, queue_size = 10, latch=True)
        self.sub_points = rospy.Subscriber("/clicked_point", PointStamped, self.lane_callback, queue_size = 1)
        self.error_queue = deque(np.zeros(ERROR_QUEUE_SIZE), maxlen = ERROR_QUEUE_SIZE)
        self.setpoint = SETPOINT
        self.error = INITIAL_ERROR
        self.Kp = KP
        self.Ki = KI
        self.Kd = KD
        self.shutdown_ = False
        self.map_size_x = 600 #cm
        self.map_size_y = 400 #cm
        self.resolution = 10 # cm
        self.lane = 1
        self.time_new = rospy.Time.now()
        self.time_old = rospy.Time.now()
        self.init_time = 0
        self.last_yaw = 0.0
        if (self.lane == 1):
            self.matrix = np.load(PATH + "matrix100cm_lane1.npy")
        else:
       	    self.matrix = np.load(PATH + "matrix100cm_lane2.npy")
	
    #Fuege eventListener hinzu, wird gecalled wenn sich rospy beendet, Funktion setzt Speed des Fahrzeuges auf 0
	rospy.on_shutdown(self.shutdown)

    def lane_callback(self, data):
    	if (self.lane == 1):
    	    self.lane = 2
    	    self.matrix = np.load(PATH + "matrix100cm_lane1.npy")
    	else:
    	    self.lane = 1
    	    self.matrix = np.load(PATH + "matrix100cm_lane2.npy")

    def callback(self, data):
        self.time_new = rospy.Time.now()
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

        x_index=np.int(x * self.resolution)
        y_index=np.int(y * self.resolution)
        
        if (x_index < 0):
            x_index = 0
        if (x_index > ((self.map_size_x/self.resolution) - 1)):
            x_index = (self.map_size_x/self.resolution) - 1

        if (y_index < 0):
            y_index = 0
        if (y_index>((self.map_size_y/self.resolution) - 1)):
            y_index=(self.map_size_y/self.resolution) - 1

        x3, y3 = self.matrix[x_index, y_index, :]
        f_x = np.cos(yaw) * x3 + np.sin(yaw) * y3
        f_y = -np.sin(yaw) * x3 + np.cos(yaw) * y3
        
        time = self.time_new - self.time_old
        dt = float(time.secs + (time.nsecs / 1000000000.0))
        print(dt)
        
        self.error = np.arctan(f_y / (2.5*f_x))
        if(self.error > np.pi):
            self.error = self.error - 2*np.pi
        if(self.error < -np.pi):
            self.error = self.error + 2*np.pi
        
        if(self.init_time and dt != 0):
            # limit error history
            self.error_queue.append(self.error * dt)
            self.integral = 0
            for e in self.error_queue:
                self.integral += e
            if(self.integral > KI_UPPER_LIMIT):
                self.integral = KI_UPPER_LIMIT
            elif(self.integral < KI_LOWER_LIMIT):
                self.integral = KI_LOWER_LIMIT
            # correct sudden huge errors
            self.derivative = (self.error - self.previous_error) / dt
            #~ self.derivative = (yaw - self.last_yaw)
            #~ if(self.derivative > np.pi):
                #~ self.derivative = self.derivative - 2*np.pi
            #~ if(self.error < -np.pi):
                #~ self.derivative = self.derivative + 2*np.pi
            #~ self.derivative = self.derivative / dt
            
            self.previous_error = self.error
            control = self.Kp * self.error + self.Ki * self.integral + self.Kd * self.derivative
        else:
            control = self.Kp * self.error

        # When the f_x_car_coordinate is negative, the car should move backwards
        if (f_x > 0):
            speed = SPEED
        else:
            speed = -SPEED 
            if (f_y > 0):
            	control = -np.pi/2
            if (f_y < 0):
            	control = np.pi/2
            
        steering = control * YAW_TO_STEERING_FACTOR + ANGLE_STRAIGHT
        #~ print("Current yaw: " + str(yaw))
        #~ print("Error: " + str(self.error))
        #~ print("Previous Error: " + str(self.previous_error))
        #~ print("Integral: " + str(self.integral))
        #~ print("Derivative: " + str(self.derivative))
        #~ print("Control: " + str(control))
        #~ print("Steering: " + str(steering) + "\n")
        
        if(steering >= MAX_ANGLE_LEFT):
            steering = UInt8(int(MAX_ANGLE_LEFT))
        elif(steering <= MAX_ANGLE_RIGHT):
            steering = UInt8(int(MAX_ANGLE_RIGHT))
        else:
            steering = UInt8(int(steering))
        
        self.steering_pub.publish(steering) 
        self.last_yaw = yaw
        self.time_old = self.time_new
        self.init_time = 1
	if not self.shutdown_:
	    self.pub_speed.publish(Int16(speed))

    #ein Listener wir bei der init() erstellt, dieser called die shutdown Funktion, wenn das script beendet wird. 
    #Setzt den Speed des autos auf 0
    def shutdown(self):
        print("shutdown!")
        self.shutdown_=True
        self.pub_speed.publish(Int16(0))
	rospy.sleep(1)

def main():
    print("Running\n")
    rospy.init_node('AljoschaTimRacer')
    ForceController()  # constructor creates publishers / subscribers
    rospy.spin()

if __name__ == '__main__':
    main()
