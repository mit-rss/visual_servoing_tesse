#!/usr/bin/env python
import rospy
from visual_servoing_tesse.msg import cone_location, parking_error
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class ParkingController():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        rospy.Subscriber("/relative_cone", cone_location, 
            self.relative_cone_callback)    
        self.drive_pub = rospy.Publisher("/tesse/drive", 
            AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/parking_error",
            parking_error, queue_size=10)

        self.parking_distance = 10 #meters
        self.relative_x = 0
        self.relative_y = 0

        rospy.logerr("initialized parking contrller")

    def relative_cone_callback(self, msg):
        

        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        angle_to_cone = np.arctan2(msg.y_pos, msg.x_pos)
        drive_cmd = AckermannDriveStamped()

        rospy.logerr(("relative cone cb!", self.relative_x, self.relative_y))

        if msg.x_pos < self.parking_distance - 0.05:# or np.abs(msg.y_pos/msg.x_pos) > .3:
            drive_cmd.drive.speed = -1.0
            drive_cmd.drive.steering_angle = -1*angle_to_cone
        elif abs(msg.x_pos - self.parking_distance < 0.05):
            drive_cmd.drive.speed = 0
        else:
            drive_cmd.drive.speed = 1.0
            drive_cmd.drive.steering_angle = 1*angle_to_cone
        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = parking_error()
        
        # Your Code Here
        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)
        error_msg.x_error = self.relative_x-self.parking_distance
        error_msg.y_error = self.relative_y
        error_msg.distance_error = \
            (self.relative_x**2 + self.relative_y**2)**.5-self.parking_distance
        self.error_pub.publish(error_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Failed to start parking controller!")
        pass