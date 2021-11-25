import argparse
import json
import sys

import cv2
import numpy as np
from compas.geometry import Transformation
from compas_fab.backends import RosClient
from roslibpy import Topic
import urllib.request

from aruco_markers import estimate_pose
from aruco_markers import load_coefficients

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Camera calibration')
    parser.add_argument('--url', default='http://192.168.1.100', type=str, required=False, help='url of the camera stream')
    parser.add_argument('--calibration_file', default='calibration_1280x720.yml', type=str, required=False, help='YML file to load calibration matrices')
    parser.add_argument('--marker_size', default=43.2, type=float, required=False, help='Real size of the printed marker')
    parser.add_argument('--ros_ip', type=str, default='192.168.1.2', required=False, help='IP address of the ROS network')

    args = parser.parse_args()

    client = RosClient(args.ros_ip)
    client.run()
    print ("ROS Connected")

    frames = Topic(client, '/camera_frames', 'std_msgs/String')
    frames.advertise()
    print ("Topic Advertised")

    K, D = load_coefficients(args.calibration_file)
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

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

            ticks += 1
            if ticks % 1000:
                ids, corners, rvecs, tvecs, marker_points, estimated_frames = estimate_pose(frame, marker_size, K, D, aruco_dict)
                if np.all(ids is not None):
                    for i in range(len(ids)):
                        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                        cv2.aruco.drawAxis(frame, K, D, rvecs[i], tvecs[i], marker_size/2)
                        if ids[i] == 25:
                            R, _jacobian = cv2.Rodrigues(rvecs[i])
                            # print("rvecs: %s \n tvecs: %s \n R: %s" % (rvecs[i], tvecs[i], R))

                            r1, r2, r3 = R.tolist()
                            r1.append(tvecs[i][0, 0, 0])
                            r2.append(tvecs[i][0, 0, 1])
                            r3.append(tvecs[i][0, 0, 2])
                            T = [r1, r2, r3, [0, 0, 0, 1]]
                            # T = T.inversed() # maybe this
                            T = Transformation.from_matrix(T)


                            print('{}'.format(T))
                            frames.publish({'data': json.dumps(T.to_data())})

            cv2.imshow('VideoStream', frame)

    finally:
        # if vcap:
        #     vcap.release()
        pass

    cv2.destroyAllWindows()

    frames.unadvertise()
    client.close()

# Launched with arguments:
# "args": ["--url", "http://192.168.1.2:80", "--calibration_file", "${workspaceFolder}/src//visual_docking/calibration_near_focus_800_600.yml", "--marker_size", "20.0", "--ros_ip", "192.168.1.4"]
