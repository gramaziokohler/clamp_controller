import argparse
import json
import sys

import cv2
import numpy as np
from compas.geometry import Transformation
from compas_fab.backends import RosClient
from roslibpy import Topic

from aruco_markers import estimate_pose
from aruco_markers import load_coefficients

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Camera calibration')
    parser.add_argument('--url', type=str, required=True, help='url of the camera stream')
    parser.add_argument('--calibration_file', type=str, required=True, help='YML file to load calibration matrices')
    parser.add_argument('--marker_size', type=float, required=True, help='Real size of the printed marker')
    parser.add_argument('--ros_ip', type=str, default='localhost', required=False, help='IP address of the ROS network')

    args = parser.parse_args()

    client = RosClient(args.ros_ip)
    client.run()

    frames = Topic(client, '/camera_frames', 'std_msgs/String')
    frames.advertise()

    K, D = load_coefficients(args.calibration_file)

    ids = None
    ticks = 0
    marker_size = args.marker_size
    try:
        vcap = cv2.VideoCapture(args.url)
        while True:
            _ret, frame = vcap.read()

            key = cv2.waitKey(1)
            if key & 0xFF == ord('q'):
                break

            # if key & 0xFF == ord('c'):
            #     ids, corners, rvecs, tvecs, marker_points, estimated_frames = estimate_pose(frame, marker_size, K, D)
            #     if np.all(ids is not None):
            #         print('Found {} markers'.format(len(ids)))
            #         for i in range(len(ids)):
            #             print(ids[i], rvecs[i], tvecs[i], corners[i].tolist())
            cv2.imshow('VideoStream', frame)

            ticks += 1
            if ticks % 1000:
                ids, corners, rvecs, tvecs, marker_points, estimated_frames = estimate_pose(frame, marker_size, K, D)
                if np.all(ids is not None):
                    for i in range(len(ids)):
                        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                        cv2.aruco.drawAxis(frame, K, D, rvecs[i], tvecs[i], marker_size/2)
                        if ids[i] == 23:
                            R, _jacobian = cv2.Rodrigues(rvecs[i])
                            R.transpose()
                            r1, r2, r3 = R.tolist()
                            r1.append(tvecs[i][0, 0, 0])
                            r2.append(tvecs[i][0, 0, 1])
                            r3.append(tvecs[i][0, 0, 2])
                            T = [r1, r2, r3, [0, 0, 0, 1]]
                            T = Transformation.from_matrix(T)
                            print('{}'.format(T))
                            frames.publish({'data': json.dumps(T.to_data())})

            cv2.imshow('VideoStream', frame)

    finally:
        if vcap:
            vcap.release()

    cv2.destroyAllWindows()

    frames.unadvertise()
    client.close()
