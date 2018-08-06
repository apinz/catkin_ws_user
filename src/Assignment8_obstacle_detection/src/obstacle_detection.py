#!/us11r/bin/env python2
import numpy as np
import rospy
# from std_msgs.msg import Image
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int16
import cv2

#Die Hoehe, bei der er starten soll, das image nach Tiefen zu durchsuchen
STARTSEARCHHEIGHT = 0

#Die Hoehe, bei der er beenden soll, das image nach Tiefen zu durchsuchen
ENDSEARCHHEIGHT = 460

#Die Breite, bei der er starten soll das image nach Tiefen zu durchsuchen
STARTSEARCHWIDTH = 160

#Die Breite, bei der er beenden soll das image nach Tiefen zu durchsuchen
ENDSEARCHWIDTH = 480

#Die Schwelle fuer die Tiefe, die im durch STARTSEARCHWIDTH und ENDSEARCHWIDTH eingegrentzen Bild summiert wurde, bei der das Auto aufhoeren soll zu fahren/die lane wechseln soll
THRESHOLD = 1000000

#Speed den das Auto fahren soll, wenn es kein Obstacle detected hat
DRIVESPEED = 150

class ObstacleDetection:
    def __init__(self):
	self.bridge = CvBridge()
	self.subDepthImage = rospy.Subscriber("/fabiankhaled/app/camera/depth/image_raw", Image, self.depthImageCallback, queue_size = 1)
        self.pubSpeed = rospy.Publisher("/AljoschaTim/speed", Int16, queue_size = 100, latch=True)

    def depthImageCallback(self, depthData):
	depthImage = self.bridge.imgmsg_to_cv2(depthData, "16UC1")
	depthArray = np.array(depthImage, dtype=np.uint16)
	#depthArray ist eine Matrix mit [460][640] <==> [Hoehe][Breite] und Werten von 0 bis 360? stellvertretend fuer die Tiefe, die gefunden wurde. Die meisten Werte sind 0
	#Loope durch die Hoehe
	depthSum = 0
	for x in depthImage[STARTSEARCHHEIGHT:ENDSEARCHHEIGHT]:
	    #Loope durch die Breite
	    for y in x[STARTSEARCHWIDTH:ENDSEARCHWIDTH]:
		depthSum += y
	print(depthSum)
	# Wenn kein Objekt davor ist, ist die Summe bei ca. 63000. Mit Objekt davor liegen wir bei ca. ueber 100000, muss allerdings noch getestet werden
	
	if THRESHOLD <= depthSum:
		SPEED = 0
	else:
		SPEED = DRIVESPEED
	print(SPEED)
	self.pubSpeed.publish(Int16(SPEED))

	# imageNp = cv2.imdecode(depthArray, cv2.IMREAD_COLOR)

def main():
    rospy.init_node('AljoschaTimObstacleDetector')
    ObstacleDetection()  # constructor creates publishers / subscribers
    rospy.spin()

if __name__ == '__main__':
    main()
