#!/usr/bin/python3

import cv2 as cv
from cv2 import aruco
import numpy as np
import math
import rospy
from geometry_msgs.msg import Quaternion

calib_data_path = "/home/shasankgunturu/movement_auv/src/Dreadnaughts-PID_test/Final PID/MultiMatrix2.npz"


calib_data = np.load(calib_data_path)
print(calib_data.files)

cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]
r_vectors = calib_data["rVector"]
t_vectors = calib_data["tVector"]

MARKER_SIZE = 17  # centimeters

marker_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)

param_markers = aruco.DetectorParameters()

center_x = []
center_y = []
z = []
rospy.init_node('detect_the_center', anonymous=False)
coordinate_publisher = rospy.Publisher("/calypso_sim/heading", Quaternion, queue_size=10)
cap = cv.VideoCapture(0)

while not rospy.is_shutdown():
    ret, frame = cap.read()
    #cv.normalize(frame, frame, 0, 255, cv.NORM_MINMAX)
    if not ret:
        break
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, reject = aruco.detectMarkers(gray_frame, marker_dict, parameters=param_markers)
    if marker_IDs:
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(marker_corners, MARKER_SIZE, cam_mat, dist_coef)
        total_markers = range(0, marker_IDs.size)
        for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
            cv.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA)
            corners = corners.reshape(4, 2)
            corners = corners.astype(int)
            top_right = corners[0].ravel()
            top_left = corners[1].ravel()
            bottom_right = corners[2].ravel()
            bottom_left = corners[3].ravel()

            # Calculating the distance
            distance = np.sqrt(tVec[i][0][2] * 2 + tVec[i][0][0] * 2 + tVec[i][0][1] ** 2)
            z.append(math.sqrt(distance*distance))# - 774.596669241*774.596669241)
            # Draw the pose of the marker
            point = cv.drawFrameAxes(frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4) 
            
            cv.putText(frame,f"id: {ids[0]} Dist: {round(distance, 2)}",top_right,cv.FONT_HERSHEY_PLAIN,1.3,(0, 0, 255),2,cv.LINE_AA,)
            cv.putText(frame,f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)} ",bottom_right,cv.FONT_HERSHEY_PLAIN,1.0,(0, 0, 255),2,cv.LINE_AA)
            center_x.append(tVec[i][0][0])
            center_y.append(tVec[i][0][1])
        sum_x = 0
        sum_y = 0
        z_coordinate = 0
        for i in total_markers:
            sum_x = sum_x + center_x[i]
            sum_y = sum_y + center_y[i]
            z_coordinate = z_coordinate + z[i]

        z_coordinate = z_coordinate/len(total_markers)
        sum_x = sum_x/len(total_markers)
        sum_y = sum_y/len(total_markers)
        coordinates = Quaternion()
        coordinates.x = sum_x
        coordinates.y = sum_y
        coordinates.z = z_coordinate
        coordinate_publisher.publish(coordinates)
        # print(sum_x)
        # print(sum_y)
        # print(z_coordinate/4)

    cv.imshow("frame", frame)
    key = cv.waitKey(1)
    if key == ord("q"):
        break
cap.release()
cv.destroyAllWindows()