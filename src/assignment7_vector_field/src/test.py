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
from geometry_msgs.msg import Point

PATH = "/home/aljoscha/uni/informatik/master/robotics/catkin_ws_user/src/assignment7_vector_field/src/"

def main():
    print("Running\n")
    matrix = np.load(PATH + "matrix100cm_lane1.npy")
    print(str(matrix))
    print(str(matrix.shape))

if __name__ == '__main__':
    main()
