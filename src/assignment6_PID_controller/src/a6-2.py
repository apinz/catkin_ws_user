#!/usr/bin/env python
import numpy as np 
import roslib
import sys
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from collections import deque
from std_msgs.msg import Int16
from std_msgs.msg import UInt8
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

USE_ROS_PID_CONTROLLER = 0
TEST_ARRAY_LENGTH = 2

MAX_ANGLE_RIGHT = 0
ANGLE_STRAIGHT = 90
MAX_ANGLE_LEFT = 180
INITIAL_ERROR = 0.0
YAW_TO_STEERING_FACTOR = 180.0 / np.pi
MAX_SPEED = 1000
KP = 3.6
KI = 1.8
KD = 0.1
SPEED = 150
ERROR_QUEUE_SIZE = 3
KI_UPPER_LIMIT = 15
KI_LOWER_LIMIT = -15

global_loc_yaw = 0.0

# to set speed manually: rostopic pub manual_control/steering std_msgs/Int16 '{data: 180}'

# This class is our python implementation of the PID controller Pseudocode from Wikipedia:
# https://en.wikipedia.org/wiki/PID_controller#Pseudocode
class PID_Controller:
    def __init__(self):
        self.previous_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0        
        self.error_queue = deque(np.zeros(ERROR_QUEUE_SIZE), maxlen = ERROR_QUEUE_SIZE)
        self.error = INITIAL_ERROR
        self.Kp = KP
        self.Ki = KI
        self.Kd = KD
        self.shutdown_ = False
        self.time_new = rospy.Time.now()
        self.time_old = rospy.Time.now()
        self.init_time = 0
        self.setpoint = 0.0
        self.steering_pub = rospy.Publisher("AljoschaTim/steering", UInt8, queue_size=1)        
        self.speed_pub = rospy.Publisher("AljoschaTim/speed", Int16, queue_size = 10, latch=True)
        self.heading_pub = rospy.Publisher("AljoschaTim/heading", Float32, queue_size = 1, latch=True)
        self.desired_yaw_pub = rospy.Publisher("AljoschaTim/desired_yaw", Float32, queue_size = 1, latch=True)
        self.error_pub = rospy.Publisher("AljoschaTim/error", Float32, queue_size = 1, latch=True)
        self.steering_sub = rospy.Subscriber("AljoschaTim/steering_setpoint", Float32, self.steering_callback, queue_size=1)
        self.odometry_sub = rospy.Subscriber("/localization/odom/1", Odometry, self.pid_callback, queue_size = 1)
        rospy.on_shutdown(self.shutdown) # on shutdown set speed to zero
    
    # /steering_setpoint has to be the desired angle
    # current state = current yaw
    def steering_callback(self, data):
        global global_loc_yaw
        
        print("Set steering setpoint to: " + str(data))
        new_setpoint = float(data.data)
        self.setpoint = new_setpoint
        
    def pid_callback(self, data):
        global global_loc_yaw
        
        self.time_new = rospy.Time.now()
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        global_loc_yaw = yaw
        self.heading_pub.publish(Float32(yaw))
        self.desired_yaw_pub.publish(Float32(self.setpoint))
        self.error_pub.publish(Float32(self.setpoint - yaw))
            
        # calculate error
        self.error = float(self.setpoint) - float(global_loc_yaw)
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

        # set speed
        speed = SPEED
        #~ if(abs(self.error) <= (np.pi / 16)):
            #~ speed = speed * 1.15
            #~ if(abs(self.integral) <= (np.pi / 16)):
                #~ speed = speed * 1.15
                #~ if(abs(self.derivative) <= (np.pi / 16)):
                    #~ speed = speed * 1.15
        #~ if speed > MAX_SPEED:
            #~ speed = MAX_SPEED
            
        # set steering
        steering = control * YAW_TO_STEERING_FACTOR + ANGLE_STRAIGHT
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

    # on shutdown set speed to zero
    def shutdown(self):
        print("Shutdown!")
        self.shutdown_ = True
        self.speed_pub.publish(Int16(0))
        rospy.sleep(1)
        
# this class uses the ROS PID controller
class SetpointStateNode:
    def __init__(self):
        self.setpoint_pub = rospy.Publisher("/setpoint", Float64, queue_size=1)
        self.state_pub = rospy.Publisher("/state", Float64, queue_size=1)
        self.control_sub = rospy.Subscriber("/control_effort", Float64, self.controlCallback, queue_size=1)
        self.local_yaw_sub = rospy.Subscriber("/AljoschaTim/yaw", Float32, self.yawCallback, queue_size=1)

    def yawCallback(self, data):
        self.state_pub.publish(Float64(data.data))
        self.setSetpoint(0)

    def controlCallback(self, data):
        print(str(data))
        
    def setSetpoint(self, setpoint):
        self.setpoint_pub.publish(Float64(setpoint))

def drive_test():
    test_angles = np.array([-2.5, 2.5])
    steering_pub = rospy.Publisher("/steering_setpoint", Float32, queue_size=1)
    idx = 0
    print(str(test_angles))
    while True:
        steering_pub.publish(test_angles[idx])
        idx = (idx + 1) % TEST_ARRAY_LENGTH
        rospy.sleep(15.)

# main
rospy.init_node('PID_setpoint_state_node', anonymous=True)
global_loc_yaw = 0
    
if(USE_ROS_PID_CONTROLLER):
    ssn = SetpointStateNode()        
else:
    pid_controller = PID_Controller()
    #~ drive_test()
                
rospy.spin()
