#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class ParkingController():

    DRIVE_TOPIC = "/tesse/drive"
    CONE_LOCATION = "/cone_relative"
    CONE_DESIRED_HEIGHT = 125
    CONE_DESIRED_ANGEL = 0.0
    def __init__(self):
        self.pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size = 5)
        self.sub = rospy.Subscriber(self.CONE_LOCATION, PointStamped, self.driveCallback)

    def driveCallback(self,msg):
        ##YOUR CODE
        pass

if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
