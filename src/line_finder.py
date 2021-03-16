#!/usr/bin/env python2

import numpy as np
import rospy
import csv
import cv2
from visual_servoing_tesse.msg import LaneLine
from cv_bridge import CvBridge, CvBridgeError
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from tf import TransformListener

class LineFinder:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("scan_topic", "/tesse/front_lidar/scan")
    SEG_CAM_TOPIC = rospy.get_param("seg_cam_topic", "/tesse/seg_cam/rgb/image_raw")
    
    LANE_LINE_TOPIC = rospy.get_param("lane_line_topic", "/lane_line")
    STEERING_RADIUS = 2*np.pi/9 #tesse car 2pi/9
    
    
    def __init__(self, lane_rgba=[0,0,0,0]):
      self.lane_rgba = lane_rgba

      #Publishers
      self._drive_publisher = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
      self._lane_line_publisher = rospy.Publisher(self.LANE_LINE_TOPIC, LaneLine, queue_size=10)

      #Subscribers
      rospy.Subscriber(self.SEG_CAM_TOPIC, Image, self._onSegCamDataReceived)
      
      #CV Bridge
      self.bridge = CvBridge() #Converts between ROS images and OpenCV Images
       
    def _onSegCamDataReceived(self, data):
        ## YOUR CODE


def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()
	
def load_lane_color(path, semantic_label):
	"""
	Helper function to extract color associated with semantic label
	"""
    with open(path) as csvfile:
      reader = csv.DictReader(csvfile, fieldnames = ['label','r','g','b','a'])
      for row in reader:
        if row['label'] == semantic_label:
          return [int(row['r']), int(row['g']), int(row['b']), int(row['a'])]

if __name__ == "__main__":
    rospy.init_node('visual_servoing_tesse')
    #csv file and semantic label to extract color
    csv_path = rospy.get_param("/line_finder/csv_path")
    lane_semantic_label = "Decal_Road_Border_left(Clone)"
    color = load_lane_color(csv_path, lane_semantic_label)
    rospy.loginfo("lane color found: " + str(color))
    
    line_finder = LineFinder(lane_rgba = color)

    rospy.spin()
