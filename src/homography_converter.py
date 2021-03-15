#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped
import tf
from tf.transformations import euler_from_quaternion
import numpy as np
import cv2
from visual_servoing_tesse.msg import cone_location, LaneLine
from visualization_msgs.msg import Marker
from sensor_msgs.msg import CameraInfo

class HomographyConverter():
    """
    Rosnode for transforming pixel-frame topics into world-frame cone poses. Subscribes
    to a segmented line in the camera frame (lane_line_topic: LaneLine) AND a point in the
    camera frame (/relative_cone_px: PointStamped). Publishes
    """
    def __init__(self):

        # Subscribe to camera transform info
        #SEG_CAM_INFO_TOPIC = rospy.get_param("seg_cam_info_topic")
        SEG_CAM_INFO_TOPIC = "/tesse/seg_cam/camera_info"
        rospy.Subscriber(SEG_CAM_INFO_TOPIC, 
            CameraInfo, self.seg_cam_info_callback)

        self.seg_intrinsic_matrix = None
        self.seg_extrinsic_matrix = np.array([[-1, 0, 0, -0.05],
                                              [0, -1, 0, 1.03],
                                              [0, 0, 1, 1.5]])

        # Subscribe to clicked point messages from rviz  
        #RELATIVE_CONE_PX_TOPIC = rospy.get_param("relative_cone_px_topic")
        #LANE_LINE_TOPIC = rospy.get_param("lane_line_topic")
        LANE_LINE_TOPIC = "/lane_line"

        rospy.Subscriber("/relative_cone_px", 
            PointStamped, self.point_callback)
        rospy.Subscriber(LANE_LINE_TOPIC, 
            LaneLine, self.line_callback)
        self.message_x = None
        self.message_y = None
        self.message_frame = "map"


        # lookahead distance (tunable control parameter)
        self.LOOKAHEAD_DISTANCE = 4.0


        self.cone_pub = rospy.Publisher("/relative_cone", 
            cone_location,queue_size=1)
        self.marker_pub = rospy.Publisher("/cone_marker",
            Marker, queue_size=1)
        self.tf_listener = tf.TransformListener()
        self.rate = rospy.Rate(10) #10 hz
        
        while not rospy.is_shutdown():
            self.publish_cone()
            self.rate.sleep()

    def publish_cone(self):
        """
        Publish the relative location of the cone
        """
        # Find out most recent relative location of cone
        if self.message_x is None:
            return
        try:
            msg_frame_pos, msg_frame_quat = self.tf_listener.lookupTransform(
                "base_link", self.message_frame, rospy.Time(0))
        except:
            return
        # Using relative transformations, convert cone in whatever frame rviz
        # was in to cone in base link (which is the control frame)
        (roll, pitch, yaw) = euler_from_quaternion(msg_frame_quat)
        cone_relative_baselink_x =\
            msg_frame_pos[0]+np.cos(yaw)*self.message_x-np.sin(yaw)*self.message_y 
        cone_relative_baselink_y =\
            msg_frame_pos[1]+np.cos(yaw)*self.message_y+np.sin(yaw)*self.message_x
        
        # Publish relative cone location
        relative_cone = cone_location()
        relative_cone.x_pos = cone_relative_baselink_x
        relative_cone.y_pos = cone_relative_baselink_y
        self.cone_pub.publish(relative_cone)

    def draw_marker(self):
        """
        Publish a marker to represent the cone in rviz
        """
        marker = Marker()
        marker.header.frame_id = self.message_frame
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = .2
        marker.scale.y = .2
        marker.scale.z = .2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = .5
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = self.message_x
        marker.pose.position.y = self.message_y
        self.marker_pub.publish(marker)

    def seg_cam_info_callback(self, msg):
        self.seg_intrinsic_matrix = np.array(msg.K).reshape((3, 3))

        # too lazy to get the closed form equation correct atm
        # pick some points in the ground plane

        PTS_GROUND_PLANE = np.array([[1.0, 0.0, 2.5, 1],
                            [-1.0, 0.0, 2.5, 1],
                            [1.0, 0.0, 3.5, 1],
                            [-1.0, 0.0, 3.5, 1],])

        PTS_IMAGE_PLANE = np.array([np.matmul(self.seg_intrinsic_matrix,
                            np.matmul(self.seg_extrinsic_matrix,
                                np.array(pt).T)
                            ) for pt in PTS_GROUND_PLANE])

        # normalize
        PTS_IMAGE_PLANE = np.array([pt / pt[-1] for pt in PTS_IMAGE_PLANE])

        #print(np.float32(PTS_GROUND_PLANE[:, [0, 2]]))
        #print(np.float32(PTS_IMAGE_PLANE[:, [0, 1]]))


        self.homography_matrix, err = cv2.findHomography(np.float32(PTS_IMAGE_PLANE[:, [0, 1]]), np.float32(PTS_GROUND_PLANE[:, [0, 2]]))

        #print("error: ", err)

        # verify homography
        PTS_BACKPROJECTED = np.matmul(self.homography_matrix,  PTS_IMAGE_PLANE.T)
        PTS_BACKPROJECTED = np.array([pt / pt[-1] for pt in PTS_BACKPROJECTED.T]).T

        #print(PTS_BACKPROJECTED)
        # verified to backproject fine


    def point_callback(self, msg):
        # get pixel coordinates
        px = msg.point.x
        py = msg.point.y

        # apply homography matrix
        if self.seg_intrinsic_matrix is None or self.seg_extrinsic_matrix is None:
            return

        '''
        # (hardcoded for now)
        H = np.array([[ 2.31630946e-05 -3.50827978e-05 -3.72600692e-01]
                      [ 1.09022607e-03 -2.89622858e-04 -3.13292098e-01]
                      [ 8.58918330e-05 -5.64162792e-03  1.00000000e+00]])
        '''

        H = np.linalg.inv(np.matmul(self.seg_intrinsic_matrix,
                                    self.seg_extrinsic_matrix))

        CAM_PT = np.array([px, py, 1]).T

        WORLD_PT_UNSCALED = np.matmul(H, CAM_PT)
        WORLD_PT = WORLD_PT_UNSCALED / WORLD_PT_UNSCALED[-1]

        x, y = WORLD_PT[0], WORLD_PT[1] # coordinates in world frame

        self.message_x = x
        self.message_y = y
        self.message_frame = "base_link"
        
        # Draw a marker for visualization
        self.draw_marker()


    def line_callback(self, msg):
        # get line parameters
        pm = msg.m
        pb = msg.b

        # get two points on the line
        px0, py0 = 200, pm*200+pb
        px1, py1 = 300, pm*300+pb

        print('p0, p1', (px0, py0), (px1, py1))

        # apply homography matrix
        if self.seg_intrinsic_matrix is None or self.seg_extrinsic_matrix is None:
            return

        
        # (hardcoded for now)
        H = np.array([[ 2.31630946e-05, -3.50827978e-05, -3.72600692e-01],
                      [ 1.09022607e-03, -2.89622858e-04, -3.13292098e-01],
                      [ 8.58918330e-05, -5.64162792e-03,  1.00000000e+00]])
        rospy.loginfo(H)

        #H2 = np.linalg.inv(np.matmul(self.seg_intrinsic_matrix,
         #                           self.seg_extrinsic_matrix))

        #rospy.loginfo(H2)

        H = self.homography_matrix
        
        CAM_PT_0 = np.array([px0, py0, 1]).T
        CAM_PT_1 = np.array([px1, py1, 1]).T

        WORLD_PT_UNSCALED_0 = np.matmul(H, CAM_PT_0)
        WORLD_PT_0 = WORLD_PT_UNSCALED_0 / WORLD_PT_UNSCALED_0[-1]

        WORLD_PT_UNSCALED_1 = np.matmul(H, CAM_PT_1)
        WORLD_PT_1 = WORLD_PT_UNSCALED_1 / WORLD_PT_UNSCALED_1[-1]

        x0, y0 = WORLD_PT_0[0], WORLD_PT_0[1] # coordinates in world frame
        x1, y1 = WORLD_PT_1[0], WORLD_PT_1[1] # coordinates in world frame

        print("w0, w1", (x0, y0), (x1, y1))

        # get back the parameterized line in the world frame
        m = (y1 - y0) / (x1 - x0)
        b = y0 - m * x0

        # find the point on the line that is LOOKAHEAD_DISTANCE ahead of the robot
        ylook = self.LOOKAHEAD_DISTANCE
        xlook = (ylook - b) / m

        rospy.loginfo(("xlook", xlook))


        self.message_x = ylook
        self.message_y = xlook * -1
        self.message_frame = "base_link"
        
        # Draw a marker for visualization
        self.draw_marker()

if __name__ == '__main__':
    try:
        rospy.init_node('HomographyConverter', anonymous=True)
        HomographyConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass