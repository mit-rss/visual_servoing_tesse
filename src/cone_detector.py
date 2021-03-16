#!/usr/bin/env python2

import cv2 as cv
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge

class ConeDetector():
    LOW_THRESHOLD = np.asarray([125, 175,137])
    HIGH_THRESHOLD = np.asarray([135, 185,147])
    SUB_TOPIC = "tesse/seg_cam/rgb/image_raw"
    SEG_IMAGE = "/threshold_msgs"
    RELATIVE_CONE="/relative_cone_px"
    BRIDGE=CvBridge()
    def __init__(self):
        self.sub = rospy.Subscriber(self.SUB_TOPIC, Image, self.image_callback)
        self.point_pub = rospy.Publisher(self.RELATIVE_CONE,PointStamped, queue_size=5)
        self.img_pub=rospy.Publisher(self.SEG_IMAGE, Image, queue_size=5)
    def image_callback(self, msg):
        image_message = self.BRIDGE.imgmsg_to_cv2(msg)

        mask = cv.inRange(image_message, self.LOW_THRESHOLD, self.HIGH_THRESHOLD)
        mask1 = cv.bitwise_and(image_message, image_message, mask=mask)
        gray = cv.cvtColor(mask1, cv.COLOR_BGR2GRAY)
        contours, hierarchy=cv.findContours(gray, cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)

        if not contours:
            bounding_box=((0,0),(0,0))
            return bounding_box
        big_contour=max(contours, key=cv.contourArea)
        x_min, y_min, w, h = cv.boundingRect(big_contour)

        
        x_max = x_min+w
        y_max = y_min+h

        mid_point = PointStamped()
        mid_point.point.x = (x_min+x_max)/2.0
        mid_point.point.y = (y_min+y_max)/2.0
        mid_point.point.z = h
        mid_point.header.frame_id = msg.header.frame_id
        
        self.point_pub.publish(mid_point)
        

if __name__ == "__main__":
    rospy.init_node("cone_detector")
    node = ConeDetector()
    rospy.spin()
