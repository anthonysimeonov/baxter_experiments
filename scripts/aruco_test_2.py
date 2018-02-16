import os
import cv2
from cv2 import aruco
import numpy as np

a = cv2.imread('./test_markers/sample_board.png')

aruco_dict = aruco.Dictionary_get( aruco.DICT_6X6_1000 )

markerLength = 40   # Here, our measurement unit is centimetre.
markerSeparation = 8   # Here, our measurement unit is centimetre.
board = aruco.GridBoard_create(2, 4, markerLength, markerSeparation, aruco_dict)

arucoParams = aruco.DetectorParameters_create()

camera_matrix =  np.array([[404.338014371, 0.0, 643.418416949], [0.0, 404.338014371, 367.605812816], [0.0, 0.0, 1.0]])
dist_coeffs = np.array([0.0191892768671, -0.0528678190185, 0.000125957631372, -0.000467705162547, 0.0132508989191])

frame = a

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

while(True):
    cv2.imshow("arucoboard", im_with_aruco_board)

    if cv2.waitKey(2) & 0xFF == ord('q'):
        break
