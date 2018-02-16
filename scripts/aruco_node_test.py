#!/usr/bin/env python
import numpy as np
import cv2
import cv2.aruco as aruco

#board parameters length = 40mm, separation = 8mm
#hard coded right_hand_camera matrix parameters
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

xBoard = 4
yBoard = 8
markerLength = 0.04 #meters
markerSep = 0.008

camMat =  np.array([[404.338014371, 0.0, 643.418416949], [0.0, 404.338014371, 367.605812816], [0.0, 0.0, 1.0]])
distCoeffs = np.array([0.0191892768671, -0.0528678190185, 0.000125957631372, -0.000467705162547, 0.0132508989191])

board = aruco.GridBoard_create(xBoard, yBoard, markerLength, markerSep, aruco_dict)

while(True):
    # Capture cap-by-cap
    cap = cv2.imread('/home/anthony/ros_ws/src/baxter_experiments/scripts/test_markers/corner_1.png')
    #print(cap.shape) #480x640
    # Our operations on the cap come here
    gray = cv2.cvtColor(cap, cv2.COLOR_BGR2GRAY)
    parameters =  aruco.DetectorParameters_create()

    #print(parameters)

    '''    detectMarkers(...)
        detectMarkers(image, dictionary[, corners[, ids[, parameters[, rejectedI
        mgPoints]]]]) -> corners, ids, rejectedImgPoints
        '''
        #lists of ids and the corners beloning to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    pose = aruco.estimatePoseBoard(corners, ids, board, camMat, distCoeffs)
    rvec = pose[1][:]
    tvec = pose[2][:]

    # print(pose)
    print(rvec)
    print(tvec)

    #It's working.
    # my problem was that the cellphone put black all around it. The alrogithm
    # depends very much upon finding rectangular black blobs

    gray = aruco.drawDetectedMarkers(gray, corners, ids, [255,0,0])
    gray = aruco.drawAxis(gray, camMat, distCoeffs, rvec, tvec, 0.1)


    #print(rejectedImgPoints)
    # Display the resulting cap
    cv2.imshow('cap',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
