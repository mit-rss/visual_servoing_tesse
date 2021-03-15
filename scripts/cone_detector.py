#!/usr/bin/env python2

import cv2 as cv
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

class ConeDetector:
    #Semantic RGB of the cone object
    SEG_LABEL = (130,180,142)
    SUB_TOPIC = "/tesse/seg_cam/rgb/image_raw"
    PUB_TOPIC = "/relative_cone"
    def __init__(self):
        self.sub = rospy.Subscriber(self.SUB_TOPIC, Image, self.callback);
        self.pub = rospy.Publisher(self.PUB_TOPIC, Point, queue_size=5)

    def callback(self, msg):
        ###YOUR CODE HERES
        pass

if __name__ == "__main__":
    rospy.init_node("cone_detector");
    node = ConeDetector()
    rospy.spin()
