import argparse
import json
import sys

import cv2
import numpy as np
from compas.geometry import Transformation
from compas_fab.backends import RosClient
from roslibpy import Topic
import urllib.request

from aruco_markers import estimate_pose, detect_markers
from aruco_markers import load_coefficients

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Camera calibration')
    parser.add_argument('--url', type=str, required=True, help='url of the camera stream')
    parser.add_argument('--calibration_file', type=str, required=True, help='YML file to load calibration matrices')
    parser.add_argument('--marker_size', type=float, required=True, help='Real size of the printed marker')
    parser.add_argument('--marker_spacing', type=float, required=True, help='Spacing between markers')
    parser.add_argument('--ros_ip', type=str, default='localhost', required=False, help='IP address of the ROS network')

    args = parser.parse_args()

    client = RosClient(args.ros_ip)
    client.run()
    print ("ROS Connected")

    frames = Topic(client, '/camera_frames', 'std_msgs/String')
    frames.advertise()
    print ("Topic Advertised")

    K, D = load_coefficients(args.calibration_file)
    dictionary_name  = cv2.aruco.DICT_4X4_50
    board = cv2.aruco.GridBoard_create(6, 4, args.marker_size, args.marker_spacing, cv2.aruco.Dictionary_get(dictionary_name))
    img = board.draw((650,430))
    cv2.imwrite("DICT_4X4_50_6x4_10_1.png", img)
    cv2.imshow("DICT_4X4_50_6x4_10_1", img)
    cv2.waitKey(0)

    ids = None
    ticks = 0
    marker_size = args.marker_size
    try:
        # cv2 Video Capture does deal with the multipart jpeg stream.
        vcap = cv2.VideoCapture(args.url)
        while True:
            _ret, frame = vcap.read() # Reads one of the multi part

            key = cv2.waitKey(1)
            if key & 0xFF == ord('q'):
                break

            cv2.imshow('VideoStream', frame)

            ticks += 1
            if ticks % 1000:
                ids, corners, frame_with_markers = detect_markers(frame, dictionary_name)

                if np.all(ids is not None):
                    rvec, tvec, marker_points = [], [], []
                    rvec = np.zeros((3,1))
                    tvec = np.zeros((3,1))
                    estimated_frames = None
                    retval, rvec, tvec = cv2.aruco.estimatePoseBoard(corners, ids, board, K, D, rvec, tvec)

                    # colorFrame = cv2.aruco.drawDetectedMarkers(colorFrame, corners, ids)
                    if(retval):
                        cv2.aruco.drawAxis(frame_with_markers, K, D, rvec, tvec, marker_size/2 )

            cv2.imshow('VideoStream', frame_with_markers)

    finally:
        # if vcap:
        #     vcap.release()
        pass

    cv2.destroyAllWindows()

    frames.unadvertise()
    client.close()

# Launched with arguments:
# "args": ["--url", "http://192.168.1.2:80", "--calibration_file", "${workspaceFolder}/src//visual_docking/calibration_near_focus_800_600.yml", "--marker_size", "20.0", "--ros_ip", "192.168.1.4"]
