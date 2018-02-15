#!/usr/bin/env python
import numpy as np
import cv2
import cv2.aruco as aruco

#board parameters length = 40mm, separation = 8mm

while(True):
    # Capture cap-by-cap
    cap = cv2.imread('/home/anthony/ros_ws/src/baxter_experiments/scripts/test_markers/marker1.jpg')
    #print(cap.shape) #480x640
    # Our operations on the cap come here
    gray = cv2.cvtColor(cap, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters =  aruco.DetectorParameters_create()

    #print(parameters)

    '''    detectMarkers(...)
        detectMarkers(image, dictionary[, corners[, ids[, parameters[, rejectedI
        mgPoints]]]]) -> corners, ids, rejectedImgPoints
        '''
        #lists of ids and the corners beloning to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    print(corners)

    #It's working.
    # my problem was that the cellphone put black all around it. The alrogithm
    # depends very much upon finding rectangular black blobs

    gray = aruco.drawDetectedMarkers(gray, corners, ids, [255,0,0])

    #print(rejectedImgPoints)
    # Display the resulting cap
    cv2.imshow('cap',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
