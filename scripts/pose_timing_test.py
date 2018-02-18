#!/usr/bin/env python
import numpy as np
import cv2
import cv2.aruco as aruco
import rospy
import sensor_msgs.msg as sensor_msgs
import geometry_msgs.msg as geometry_msgs
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
from transforms3d.euler import euler2quat
import time
import matplotlib.pyplot as plt
import std_msgs.msg as std_msgs

class estimatePose:
    def __init__(self):
        #subscribers
        self.image_sub = rospy.Subscriber('/cameras/right_hand_camera/image', sensor_msgs.Image, self.img_handler)
        self.info_sub = rospy.Subscriber('/cameras/right_hand_camera/camera_info', sensor_msgs.CameraInfo, self.info_handler)
        self.cycle_sub = rospy.Subscriber('/board_pose/cycle_on', std_msgs.Bool, self.cycle_handler)

        #publishers
        self.image_markers_pub = rospy.Publisher('/board_pose/image_markers', sensor_msgs.Image, queue_size = 0)
        self.pose_pub = rospy.Publisher('/board_pose/pose', geometry_msgs.PoseStamped, queue_size = 0)
        self.cycle_time_pub = rospy.Publisher('/board_pose/cycle_time_2', std_msgs.Time, queue_size = 0)

        #initialize self variables
        self.bridge = CvBridge()
        self.camMat = None
        self.distCoeffs = None

        #empty published topics
        self.pose_msg = geometry_msgs.PoseStamped()
        self.cycle_time_msg = std_msgs.Time()

    def info_handler(self, data):
        #get information from camera info topic and map it to inputs for the aruco functions
        raw_K = np.array(data.K)
        self.camMat = np.reshape(raw_K, (3,3))
        self.distCoeffs = np.array(data.D)

    def img_handler(self, data):
        try:
            #convert sensor_msgs to cv2 image
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
        #hard coded aruco board parameters
        aruco_dict = aruco.Dictionary_get( aruco.DICT_6X6_1000 )

        markerLength = 12.35   # Here, our measurement unit is centimetre.
        markerSeparation = 2.47   # Here, our measurement unit is centimetre.
        board = aruco.GridBoard_create(1, 2, markerLength, markerSeparation, aruco_dict)

        arucoParams = aruco.DetectorParameters_create()

        camera_matrix =  np.array([[404.338014371, 0.0, 643.418416949], [0.0, 404.338014371, 367.605812816], [0.0, 0.0, 1.0]])
        dist_coeffs = np.array([0.0191892768671, -0.0528678190185, 0.000125957631372, -0.000467705162547, 0.0132508989191])

        frame = image

        frame_remapped = frame #cv2.remap(frame, map1, map2, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)     # for fisheye remapping
        frame_remapped_gray = cv2.cvtColor(frame_remapped, cv2.COLOR_BGR2GRAY)

        corners, ids, rejectedImgPoints = aruco.detectMarkers(frame_remapped_gray, aruco_dict, parameters=arucoParams)  # First, detect markers
        aruco.refineDetectedMarkers(frame_remapped_gray, board, corners, ids, rejectedImgPoints)


        if len(ids)>0: # if there is at least one marker detected
            im_with_aruco_board = aruco.drawDetectedMarkers(frame_remapped, corners, ids, (0,255,0))
            retval, rvec, tvec = aruco.estimatePoseBoard(corners, ids, board, camera_matrix, dist_coeffs)  # posture estimation from a diamond
            if retval != 0:
                im_with_aruco_board = aruco.drawAxis(im_with_aruco_board, camera_matrix, dist_coeffs, rvec, tvec, 100)  # axis length 100 can be changed according to your requirement
        else:
            im_with_aruco_board = frame_remapped

        image_markers = im_with_aruco_board

        poseQuaternion = euler2quat(rvec[0], rvec[1], rvec[2])

        #convert to ROS msg types
        now = rospy.Time.now()
        self.pose_msg.header.seq = 0
        self.pose_msg.header.stamp = now
        self.cycle_time_msg.data = now

        #data
        self.pose_msg.header.frame_id = 'camera_frame'
        self.pose_msg.pose.position.x = tvec[0]
        self.pose_msg.pose.position.y = tvec[1]
        self.pose_msg.pose.position.z = tvec[2]
        self.pose_msg.pose.orientation.x = poseQuaternion[1]
        self.pose_msg.pose.orientation.y = poseQuaternion[2]
        self.pose_msg.pose.orientation.z = poseQuaternion[3]
        self.pose_msg.pose.orientation.w = poseQuaternion[0]


        #publish
        self.pose_pub.publish(self.pose_msg)
        # image_markers = image

        try:
            self.image_markers_pub.publish(self.bridge.cv2_to_imgmsg(image_markers, 'bgr8'))
        except CvBridgeError as e:
            print(e)

    def cycle_handler(self, data):
        print(data)
        if data:
            self.cycle_time_pub.publish(self.cycle_time_msg)


if __name__== '__main__':
    rospy.init_node('pose_estimator', anonymous = True)
    rate = rospy.Rate(10)
    pose_estimator = estimatePose()
    while not rospy.is_shutdown():
        rate.sleep()
