#!/usr/bin/env python2
import numpy as np
import rospy
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

#OBSTACLE DETECTION SETTINGS
#VALUES FROM CAMERA INTO CM DISTANCE
#500 => 30 cm, 700 => 50cm
THRESHOLD = 800
MIN_COUNTER = 400
START_SEARCH_HEIGHT = 215
END_SEARCH_HEIGHT = 245
START_SEARCH_WIDTH = 305
END_SEARCH_WIDTH = 335

class SpeedAndStop:
    def __init__(self):
	self.shutdown_ = False
	self.bridge = CvBridge()
        self.speed_pub = rospy.Publisher("AljoschaTim/speed", Int16, queue_size = 10, latch=True)
	self.subDepthImage = rospy.Subscriber("AljoschaTim/app/camera/depth/image_raw", Image, self.depthImageCallback, queue_size = 10)
        rospy.on_shutdown(self.shutdown) # on shutdown set speed to zero

    def depthImageCallback(self, depthData):
	if self.shutdown_:
		return
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
				self.shutdown()
				return
	self.speed_pub.publish(Int16(260))
	

    #ein Listener wir bei der init() erstellt, dieser called die shutdown Funktion, wenn das script beendet wird. 
    #Setzt den Speed des autos auf 0
    def shutdown(self):
        print("shutdown!")
        self.shutdown_ = True
        self.speed_pub.publish(Int16(0))
        rospy.sleep(1)

def main():
    rospy.init_node('AljoschaTimRacer')
    SpeedAndStop()  # constructor creates publishers / subscribers
    rospy.spin()

if __name__ == '__main__':
    main()
