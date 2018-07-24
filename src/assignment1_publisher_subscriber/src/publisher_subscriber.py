#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32

def callback(msg):
    publisher.publish('I heard: ' + str(msg))
       
rospy.init_node("yaw_subscriber")
publisher = rospy.Publisher("/assignment1_publisher_subsriber", String, queue_size=10)
rospy.Subscriber("/yaw", Float32, callback)
rospy.spin()
