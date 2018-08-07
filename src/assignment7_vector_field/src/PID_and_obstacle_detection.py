#!/usr/bin/env python2
import numpy as np
import rospy
import tf
import roslib
import sys
import time
from collections import deque
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Int16, UInt8
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

MAX_ANGLE_RIGHT = 0
ANGLE_STRAIGHT = 90
MAX_ANGLE_LEFT = 180
INITIAL_ERROR = 0.0
YAW_TO_STEERING_FACTOR = 180.0 / np.pi
MAX_SPEED = 1000
WHEELBASE = 0.28 # 28cm

KP = 3.6
KI = 1.8
KD = 0.1
SPEED = 320

# MORE INTEGRAL SETTINGS
ERROR_QUEUE_SIZE = 3
KI_UPPER_LIMIT = 15
KI_LOWER_LIMIT = -15

#OBSTACLE DETECTION SETTINGS
#VALUES FROM CAMERA INTO CM DISTANCE
#500 => 30 cm, 700 => 50cm
THRESHOLD = 900
MIN_COUNTER = 400
STEERING_ADD = 0.25
#delay until the next lane swap can occur
DONT_SWAP_LANE_DELAY = 1500
START_SEARCH_HEIGHT = 215
END_SEARCH_HEIGHT = 245
START_SEARCH_WIDTH = 305
END_SEARCH_WIDTH = 335

PATH = "/root/catkin_ws_user/src/assignment7_vector_field/src/"

class ForceController:
    def __init__(self):
	self.waitWithSwapLaneUntil = 0
	self.bridge = CvBridge()
	self.steeringPush = 0
	
        self.previous_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0        
        self.error_queue = deque(np.zeros(ERROR_QUEUE_SIZE), maxlen = ERROR_QUEUE_SIZE)
        self.error = INITIAL_ERROR
        self.Kp = KP
        self.Ki = KI
        self.Kd = KD
        self.shutdown_ = False
        self.map_size_x = 600 #cm
        self.map_size_y = 400 #cm
        self.resolution = 10 # cm
        self.time_new = rospy.Time.now()
        self.time_old = rospy.Time.now()
        self.init_time = 0
        self.last_yaw = 0.0
        self.matrix = np.load(PATH + "matrix100cm_lane1.npy")
        self.lane = 1 
        self.steering_pub = rospy.Publisher("AljoschaTim/steering", UInt8, queue_size=1)
        self.speed_pub = rospy.Publisher("AljoschaTim/speed", Int16, queue_size = 10, latch=True)
        self.odometry_sub = rospy.Subscriber("/localization/odom/1", Odometry, self.odometry_callback, queue_size = 1)
	self.subDepthImage = rospy.Subscriber("AljoschaTim/app/camera/depth/image_raw", Image, self.depthImageCallback, queue_size = 10)
        rospy.on_shutdown(self.shutdown) # on shutdown set speed to zero

    def lane_swap(self):
	self.steeringPush = STEERING_ADD
	self.waitWithSwapLaneUntil = int(round(time.time() * 1000)) + DONT_SWAP_LANE_DELAY
        if (self.lane == 1):
            print("Switch to outer lane.")
            self.lane = 2
            self.matrix = np.load(PATH + "matrix100cm_lane1.npy")
        else:
            print("Switch to inner lane.")
            self.lane = 1
            self.matrix = np.load(PATH + "matrix100cm_lane2.npy")

    def odometry_callback(self, data):
        self.time_new = rospy.Time.now()
        
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

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

        x3, y3 = self.matrix[x_index, y_index, :]
        f_x = np.cos(yaw) * x3 + np.sin(yaw) * y3
        f_y = -np.sin(yaw) * x3 + np.cos(yaw) * y3
        
        # calculate error
        self.error = np.arctan(f_y / (2.5 * f_x))
        if(self.error > np.pi):
            self.error = self.error - 2 * np.pi
        if(self.error < -np.pi):
            self.error = self.error + 2 * np.pi
        
        time = self.time_new - self.time_old
        dt = float(time.secs + (time.nsecs / 1000000000.0))
        
        if(self.init_time and dt > 0.01):
            # limit error history, calculate integral control
            self.error_queue.append(self.error * dt)
            self.integral = 0
            for e in self.error_queue:
                self.integral += e
            if(self.integral > KI_UPPER_LIMIT):
                self.integral = KI_UPPER_LIMIT
            elif(self.integral < KI_LOWER_LIMIT):
                self.integral = KI_LOWER_LIMIT
            
            # correct sudden huge errors, calculate differential control
            #~ self.derivative = (yaw - self.last_yaw)
            self.derivative = (self.error - self.previous_error)
            if(abs(self.derivative) > 0.001):
                if(self.derivative > np.pi):
                    self.derivative = self.derivative - 2 * np.pi
                if(self.error < -np.pi):
                    self.derivative = self.derivative + 2 * np.pi
                self.derivative = self.derivative / dt
                control = self.Kp * self.error + self.Ki * self.integral + self.Kd * self.derivative
            else:
                control = self.Kp * self.error + self.Ki * self.integral
            
            self.previous_error = self.error
            
        else:
            control = self.Kp * self.error

        # When the f_x_car_coordinate is negative, the car should move backwards
        if (f_x > 0):
            speed = SPEED
            
            if(abs(self.error) <= (np.pi / 16)):
                speed = speed * 1.15
                if(abs(self.integral) <= (np.pi / 16)):
                    speed = speed * 1.15
                    if(abs(self.derivative) <= (np.pi / 16)):
                        speed = speed * 1.15
            
            if speed > MAX_SPEED:
                speed = MAX_SPEED

        else:
            speed = int(-SPEED / 2)
            if (f_y > 0):
                control = -np.pi / 2
            if (f_y < 0):
                control = np.pi / 2
            
        steering = control * YAW_TO_STEERING_FACTOR + ANGLE_STRAIGHT

	if steering > ANGLE_STRAIGHT:
		steering = (1 + self.steeringPush) * steering
	elif steering < ANGLE_STRAIGHT:
		steering = (1 - self.steeringPush) * steering
        
        if(steering >= MAX_ANGLE_LEFT):
            steering = UInt8(int(MAX_ANGLE_LEFT))
        elif(steering <= MAX_ANGLE_RIGHT):
            steering = UInt8(int(MAX_ANGLE_RIGHT))
        else:
            steering = UInt8(int(steering))
        
        self.last_yaw = yaw
        self.time_old = self.time_new
        self.init_time = 1
        
        if not self.shutdown_:
            self.steering_pub.publish(steering)
            self.speed_pub.publish(Int16(speed))

    def depthImageCallback(self, depthData):	
	if int(round(time.time() * 1000)) <= self.waitWithSwapLaneUntil:
		return
	else:
		self.steeringPush = 0
	depthImage = self.bridge.imgmsg_to_cv2(depthData, "16UC1")
	x = np.array(depthImage, dtype=np.uint16)
	#depthArray ist eine Matrix mit [460][640] <==> [Hoehe][Breite] und Werten von 0 bis 360? stellvertretend fuer die Tiefe, die gefunden wurde. Die meisten Werte sind 0
	#Loope durch die Hoehe
	counter = 0
	for heightArray in x[START_SEARCH_HEIGHT:END_SEARCH_HEIGHT]:
		for y in heightArray[START_SEARCH_WIDTH:END_SEARCH_WIDTH]:
			if y != 0 and y <= THRESHOLD:
				counter += 1
			if counter >= MIN_COUNTER:
				self.lane_swap()
				return

    #ein Listener wir bei der init() erstellt, dieser called die shutdown Funktion, wenn das script beendet wird. 
    #Setzt den Speed des autos auf 0
    def shutdown(self):
        print("shutdown!")
        self.shutdown_ = True
        self.speed_pub.publish(Int16(0))
        rospy.sleep(1)

def main():
    print("Running\n")
    rospy.init_node('AljoschaTimRacer')
    ForceController()  # constructor creates publishers / subscribers
    rospy.spin()

if __name__ == '__main__':
    main()
