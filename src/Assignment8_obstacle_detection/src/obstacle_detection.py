#!/us11r/bin/env python2
import numpy as np
import rospy
# from std_msgs.msg import Image
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

#Die Breite, bei der er starten soll das image nach Tiefen zu durchsuchen
STARTSEARCHWIDTH = 160
#Die Breite, bei der er beenden soll das image nach Tiefen zu durchsuchen
ENDSEARCHWIDTH = 480

class ObstacleDetection:
    def __init__(self):
	self.bridge = CvBridge()
	self.subDepthImage = rospy.Subscriber("/fabiankhaled/app/camera/depth/image_raw", Image, self.depthImageCallback, queue_size = 1)

    def depthImageCallback(self, depthData):
	print("Hallo Welt!")
	depthImage = self.bridge.imgmsg_to_cv2(depthData, "16UC1")
	depthArray = np.array(depthImage, dtype=np.uint16)
	#DepthArray ist eine Matrix mit [460][640] <==> [Höhe][Breite] und Werten von 0 bis 360? stellvertretend für die Tiefe, die gefunden wurde. Die meisten Werte sind 0
	#Loope durch die Höhe
	for x in depthArray:
		#Loope durch die Breite
		for y in x[STARTSEARCHWIDTH:ENDSEARCHWIDTH]:
		
	print((depthArray[240]))
	print(len(depthArray[240]))
	imageNp = cv2.imdecode(depthArray, cv2.IMREAD_COLOR)

def main():
    rospy.init_node('AljoschaTimObstacleDetector')
    ObstacleDetection()  # constructor creates publishers / subscribers
    rospy.spin()

if __name__ == '__main__':
    main()
