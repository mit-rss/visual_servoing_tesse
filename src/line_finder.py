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
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from tf import TransformListener

class LineFinder:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("scan_topic", "/tesse/front_lidar/scan")
    DRIVE_TOPIC = rospy.get_param("drive_topic", "/tesse/drive")
    ODOMETRY_TOPIC = rospy.get_param("odometry_topic", "/tesse/odom")
    BODY_FRAME_TOPIC = rospy.get_param("tesse/tesse_ros_bridge/body_frame_id_gt", "base_link")
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

      #Initialize AckermannDriveMsg
      self._aDrive = AckermannDriveStamped()
      self._aDrive.header.frame_id = self.BODY_FRAME_TOPIC
      self._aDrive.drive.speed = 0
      self._aDrive.drive.steering_angle = 0
      self._aDrive.drive.acceleration = 0
      self._aDrive.drive.jerk = 0
      self._aDrive.drive.steering_angle_velocity = 0
      
      #CV Bridge
      self.bridge = CvBridge() #Converts between ROS images and OpenCV Images
       
    def _onSegCamDataReceived(self, data):
        img_height = data.height
        img_width = data.width
        img_step = data.step
        try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
          rospy.loginfo(e)
        #get rgb values of lane marker and reverse list
        lower = np.array(self.lane_rgba[:-1][::-1])
        upper = np.array(self.lane_rgba[:-1][::-1])
        #cut out sky
        height, width, _ = cv_image.shape
        upperbound = int(height*3/5) #in terms of physical image up
        lowerbound = int(height*5/5) #all the way to the bottom of the image
        cv_image[:upperbound,:,:] = 0
        cv_image[lowerbound:,:,:] = 0
        #mask parts of image with only lane color
        lane_mask = cv2.inRange(cv_image,lower,upper)
        kernel = np.ones((5,5),np.uint8)
        #exagerate lane markings
        dilated = cv2.dilate(lane_mask,kernel,iterations = 1)
        #find the lines in the image
        lines = cv2.HoughLines(dilated, 1, np.pi/45, 50) #tunable parameters
        rospy.loginfo("num_lines," + str(len(lines)))
        #consolidate the lines using an averager
        mean_m, mean_b = hough_line_averager(lines, cv_image)
        
        #draw average line defined by m and b y=mx+b form
        y_max = height #bottom of image
        y_min = height*3/5 #arbitrary right now
        mean_x1 = (y_max - mean_b)/mean_m
        mean_y1 = y_max
        mean_x2 = (y_min - mean_b)/mean_m
        mean_y2 = y_min
        #rospy.loginfo("mean x1,y1" + str((mean_x1, mean_y1)))
        #rospy.loginfo("mean x2,y2" + str((mean_x2, mean_y2)))
        rospy.loginfo("mean_m" + str(mean_m))
        rospy.loginfo("mean_b" + str(mean_b))
        cv2.line(cv_image,(int(mean_x1),int(mean_y1)),(int(mean_x2),int(mean_y2)),(0,0,255),3) #bgr red
        #image_print(cv_image)
        #setpoint if we wanted to hardcode a pixel on the line to follow
        y_setpoint = height*4/5
        x_setpoint = (y_setpoint - mean_b)/mean_m
        
        lane_line = LaneLine()
        lane_line.m = mean_m
        lane_line.b = mean_b
        self._lane_line_publisher.publish(lane_line)
        
    def drive_pub(self, speed, angle):
      self._aDrive.header.stamp = rospy.get_rostime()
      self._aDrive.drive.speed = speed
      self._aDrive.drive.steering_angle = angle
      self._drive_publisher.publish(self._aDrive)
      
def hough_line_averager(lines, img):
  ms = []
  bs = []
  for line in lines:
    rho,theta = line[0]
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a*rho
    y0 = b*rho
    x1 = int(x0 + 3000*(-b))
    y1 = int(y0 + 3000*(a))
    x2 = int(x0 - 3000*(-b))
    y2 = int(y0 - 3000*(a))

    # calculate slope and intercept
    m = float(y2-y1)/(x2-x1+.0001) #avoid divide by 0
    b = y1 - x1*m
    ms.append(m)
    bs.append(b)
    cv2.line(img,(x1,y1),(x2,y2),(0,255,0),1) #draw the hough lines in green color for debugging
  return np.average(ms), np.average(bs)


def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()
	
def load_lane_color(path, semantic_label):
    with open(path) as csvfile:
      reader = csv.DictReader(csvfile, fieldnames = ['label','r','g','b','a'])
      for row in reader:
        if row['label'] == semantic_label:
          return [int(row['r']), int(row['g']), int(row['b']), int(row['a'])]

if __name__ == "__main__":
    rospy.init_node('visual_servoing_tesse')
    csv_path = rospy.get_param("/line_finder/csv_path","/tesse_windridge_city_scene_segmentation_mapping.csv")
    lane_semantic_label = "Decal_Road_Border_left(Clone)"
    color = load_lane_color(csv_path, lane_semantic_label)
    rospy.loginfo("lane color found: " + str(color))
    
    line_finder = LineFinder(lane_rgba = color)

    rospy.spin()
