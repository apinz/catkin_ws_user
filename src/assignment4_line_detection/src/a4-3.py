#!/usr/bin/env python
import numpy as np
import sys
import roslib
import rospy
import cv2

PATH = "./hsv_filtered.png"
EROSION_KERNEL = np.ones((1,24), np.uint8)
DILATE_KERNEL = np.ones((2,24), np.uint8)
#~ EDGE_KERNEL = np.array([[1.0,1.0,1.0], [1.0,-8.0,1.0], [1.0,1.0,1.0]])
LINE_WIDTH = 20

# http://www.roguebasin.com/index.php?title=Bresenham%27s_Line_Algorithm#Python
def get_line(start, end):
    """Bresenham's Line Algorithm
    Produces a list of tuples from start and end
 
    >>> points1 = get_line((0, 0), (3, 4))
    >>> points2 = get_line((3, 4), (0, 0))
    >>> assert(set(points1) == set(points2))
    >>> print points1
    [(0, 0), (1, 1), (1, 2), (2, 3), (3, 4)]
    >>> print points2
    [(3, 4), (2, 3), (1, 2), (1, 1), (0, 0)]
    """
    # Setup initial conditions
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1
 
    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx)
 
    # Rotate line
    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2
 
    # Swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
 
    # Recalculate differentials
    dx = x2 - x1
    dy = y2 - y1
 
    # Calculate error
    error = int(dx / 2.0)
    ystep = 1 if y1 < y2 else -1
 
    # Iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = (y, x) if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx
 
    # Reverse the list if the coordinates were swapped
    if swapped:
        points.reverse()
    return points


# p1 and p2 are the endpoints of a line, lwidth is the width of
# the rectangles above and below this line.
def getArea(p1, p2, lwidth):
    area = []    
    #~ for i in range(lwidth):
        #~ points = get_line((p1[0], p1[1] + i), p2)
    #~ m = float(p2[1] - p1[1]) / float(p2[0] - p1[0])                
    return area

def areaScan(area):
    return pointSet

def ransac(img):
    p1 = (40, 400)
    p2 = (600, 440)    
    vp = np.subtract(p2, p1)
    vp_mag = np.sqrt(vp.dot(vp))
    vp_norm = np.divide(vp, vp_mag)
    vp_rot_1 = np.array([vp_norm[1], -vp_norm[0]])
    vp_rot_2 = np.array([-vp_norm[1], vp_norm[0]])
    q1 = tuple([int(p1[0] + vp_rot_1[0] * LINE_WIDTH), int(p1[1] + vp_rot_1[1] * LINE_WIDTH)])
    q2 = tuple([int(p2[0] + vp_rot_1[0] * LINE_WIDTH), int(p2[1] + vp_rot_1[1] * LINE_WIDTH)])
    r1 = tuple([int(p1[0] + vp_rot_2[0] * LINE_WIDTH), int(p1[1] + vp_rot_2[1] * LINE_WIDTH)])
    r2 = tuple([int(p2[0] + vp_rot_2[0] * LINE_WIDTH), int(p2[1] + vp_rot_2[1] * LINE_WIDTH)])
    
    m = float(p2[1] - p1[1]) / float(p2[0] - p1[0])
    b = p2[1] - m*p2[0]
    
    getArea(p1, p2, LINE_WIDTH)
    #~ print(str(points))
    #~ areaScan()
    
    cv2.line(img, p1, p2, (255,0,0), 2)    
    cv2.line(img, q1, q2, (255,0,0), 2) 
    cv2.line(img, r1, r2, (255,0,0), 2) 
    cv2.imshow("Image window", img)
    cv2.waitKey(5000)    

def main(args):
    print("Running\n")
    rospy.init_node('ransac_node', anonymous=True)
    
    img = cv2.imread(PATH)
    
    #~ filtered_img = cv2.filter2D(img, -1, EDGE_KERNEL)
    erosion = cv2.erode(img, EROSION_KERNEL, iterations = 1)
    dilation = cv2.dilate(erosion, DILATE_KERNEL, iterations = 3)
    
    # Use RANSAC to estimate a linear model (y = mx+b) on the image and obtain the two equations of the lines on the road.
    ransac(dilation)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
