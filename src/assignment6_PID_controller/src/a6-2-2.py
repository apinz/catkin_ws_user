#!/usr/bin/env python
import numpy as np 
import roslib
import sys
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import Float64

SPEED_RPM = 200
ANGLE_LEFT = 30
ANGLE_STRAIGHT = 90
ANGLE_RIGHT = 150
SETPOINT = 93
LOOP_FREQ = 4
INITIAL_ERROR = 0
KP = 5
KI = 0
KD = 0

# to set speed manually: rostopic pub manual_control/steering std_msgs/Int16 '{data: 180}'

# Algorithm from https://en.wikipedia.org/wiki/PID_controller#Pseudocode
class PID_Controller:
    def __init__(self):
        self.previous_error = 0
        self.integral = 0
        self.derivative = 0
        self.yaw_sub = rospy.Subscriber("/Kaenel/yaw", Float32, self.yawCallback, queue_size=1)
        self.steering_sub = rospy.Subscriber("/steering_setpoint", Int16, self.steeringCallback, queue_size=1)
        self.steering_pub = rospy.Publisher("manual_control/steering", Int16, queue_size=1)
        self.dt = LOOP_FREQ
        self.rate = rospy.Rate(self.dt)
        self.setpoint = SETPOINT
        self.error = INITIAL_ERROR
        self.Kp = KP
        self.Ki = KI
        self.Kd = KD
    
    # current state = current yaw
    def yawCallback(self, yaw):
        while not rospy.is_shutdown():
            dt = (1 / self.dt) # T = 1/f
            self.error = self.setpoint - yaw.data
            self.integral = self.integral + self.error * dt
            self.derivative = (self.error - self.previous_error) / dt
            seld.previous_error = self.error
            control = self.Kp * self.error + self.Ki * self.integral + seld-Kd * seld.derivative
            self.steering_pub(control)
            self.rate.sleep()
    
    # /steering_setpoint has to be the desired angle
    def steeringCallback(self, data):
        self.setpoint = data

class SetpointStateNode:
    def __init__(self):
        self.setpoint_pub = rospy.Publisher("/setpoint", Float64, queue_size=1)
        self.state_pub = rospy.Publisher("/state", Float64, queue_size=1)
        self.control_sub = rospy.Subscriber("/control_effort", Float64, self.controlCallback, queue_size=1)
        self.yaw_sub = rospy.Subscriber("/Kaenel/yaw", Float32, self.yawCallback, queue_size=1)

    def yawCallback(self, data):
        self.state_pub.publish(Float64(data.data))
        self.setSetpoint(0)

    def controlCallback(self, data):
        print(str(data))
        
    def setSetpoint(self, setpoint):
        self.setpoint_pub.publish(Float64(setpoint))

def main(args):
    print("Running\n")
    rospy.init_node('PID_setpoint_state_node', anonymous=True)
    ssn = SetpointStateNode()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)

