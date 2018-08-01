#!/usr/bin/env python
import numpy as np 
import roslib
import sys
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float64

class SetpointStateNode:

    def __init__(self):
        self.setpoint_pub = rospy.Publisher("/setpoint/data", Float64, queue_size=1)
        self.state_pub = rospy.Publisher("/state/data", Float64, queue_size=1)
        self.control_sub = rospy.Subscriber("/control_effort/data", Float64, self.callback, queue_size=1)
        self.yaw_sub = rospy.Subscriber("/yaw", Float32, self.callback, queue_size=1)

    def yawCallback(self,data):
        self.state_pub.publish(data)

    def controlCallback(self,data):
        # TODO: Control the car

def main(args):
    print("Running\n")
    rospy.init_node('PID_setpoint_state_node', anonymous=True)
    ssn = SetpointStateNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)

