import numpy as np
import cv2
import cv2.aruco as aruco


aruco_dict = aruco.Dictionary_get( aruco.DICT_6X6_1000 )
print(aruco_dict)

markerLength = 40   # Here, our measurement unit is centimetre.
markerSeparation = 8   # Here, our measurement unit is centimetre.
num_x = 4
num_y = 6
board = aruco.GridBoard_create(num_x, num_y, markerLength, markerSeparation, aruco_dict)

img = aruco.drawMarker(aruco_dict, 2, 700)
img2 = aruco.drawPlanarBoard(board, (11000,int((11000/num_x)*num_y)))
cv2.imwrite("test_marker4.jpg", img2)



