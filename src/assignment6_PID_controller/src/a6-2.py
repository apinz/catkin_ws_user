#!/usr/bin/env python
import numpy as np 
import roslib
import sys
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float64

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

