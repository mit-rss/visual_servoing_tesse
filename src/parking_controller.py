#!/usr/bin/env python
import rospy
from visual_servoing_tesse.msg import cone_location, parking_error
from geometry_msgs.msg import PointStamped
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class ParkingController():

    DRIVE_TOPIC = "/tesse/drive"
    CONE_LOCATION = "/relative_cone"
    PARKING_ERROR_TOPIC = "/parking_error"
    def __init__(self):
        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size = 5)
        self.sub = rospy.Subscriber(self.CONE_LOCATION, cone_location, self.relative_cone_callback)
	self.error_pub = rospy.Publisher(self.PARKING_ERROR_TOPIC, parking_error, queue_size = 5)
    def relative_cone_callback(self,msg):
        ##YOUR CODE
        pass

if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
