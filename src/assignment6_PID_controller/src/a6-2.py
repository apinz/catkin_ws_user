#!/usr/bin/env python
import numpy as np 
import roslib
import sys
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import UInt8
from std_msgs.msg import Float32
from std_msgs.msg import Float64

USE_ROS_PID_CONTROLLER = 0
TEST_ARRAY_LENGTH = 2
DEBUG = 1

SPEED_RPM = 200
MAX_ANGLE_RIGHT = 0
ANGLE_STRAIGHT = 93
MAX_ANGLE_LEFT = 180
SETPOINT = -2.5
LOOP_FREQ = 10
INITIAL_ERROR = 0.0
KP = 5.0
KI = 0.05
KD = 0.1
UPPER_LIMIT = 10
LOWER_LIMIT = -10

# to set speed manually: rostopic pub manual_control/steering std_msgs/Int16 '{data: 180}'

# This class is our python implementation of the PID controller Pseudocode from Wikipedia:
# https://en.wikipedia.org/wiki/PID_controller#Pseudocode
class PID_Controller:
    def __init__(self):
        self.previous_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.yaw_sub = rospy.Subscriber("/AljoschaTim/yaw", Float32, self.yawCallback, queue_size=1)
        self.steering_sub = rospy.Subscriber("/steering_setpoint", Float32, self.steeringCallback, queue_size=1)
        self.steering_pub = rospy.Publisher("AljoschaTim/steering", UInt8, queue_size=1)
        self.dt = LOOP_FREQ
        self.rate = rospy.Rate(self.dt)
        self.setpoint = SETPOINT
        self.error = INITIAL_ERROR
        self.Kp = KP
        self.Ki = KI
        self.Kd = KD
    
    # current state = current yaw
    def yawCallback(self, yaw):
        dt = 1.0 / self.dt # T = 1/f
        self.error = float(self.setpoint) - float(yaw.data)
        self.integral = float(self.integral) + float(self.error) * dt
        if(self.integral > UPPER_LIMIT):
            self.integral = UPPER_LIMIT
        elif(self.integral < LOWER_LIMIT):
            self.integral = LOWER_LIMIT
        self.derivative = (self.error - self.previous_error) / dt
        self.previous_error = self.error
        control = UInt8(int(self.Kp * self.error + self.Ki * self.integral + self.Kd * self.derivative + ANGLE_STRAIGHT))
        self.steering_pub.publish(control)
        
        if(DEBUG):
            print("Current yaw: " + str(yaw.data))
            print("Setpoint: " + str(self.setpoint))
            print("Error: " + str(self.error))
            print("Previous Error: " + str(self.previous_error))
            print("Integral: " + str(self.integral))
            print("Derivative: " + str(self.derivative))
            print("Control: " + str(control) + "\n")
        
        self.rate.sleep()
    
    # /steering_setpoint has to be the desired angle
    def steeringCallback(self, data):
        print("Set steering setpoint to: " + str(data))
        new_setpoint = float(data.data)
        self.setpoint = new_setpoint

# this class uses the ROS PID controller
class SetpointStateNode:
    def __init__(self):
        self.setpoint_pub = rospy.Publisher("/setpoint", Float64, queue_size=1)
        self.state_pub = rospy.Publisher("/state", Float64, queue_size=1)
        self.control_sub = rospy.Subscriber("/control_effort", Float64, self.controlCallback, queue_size=1)
        self.yaw_sub = rospy.Subscriber("/AljoschaTim/yaw", Float32, self.yawCallback, queue_size=1)

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
        rospy.sleep(5.)

def main(args):
    print("Running\n")    
    rospy.init_node('PID_setpoint_state_node', anonymous=True)
    
    if(USE_ROS_PID_CONTROLLER):
        ssn = SetpointStateNode()        
    else:
        pid_controller = PID_Controller()
        drive_test()
                
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)

