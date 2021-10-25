import argparse
import json
import time

import cv2
import numpy as np
from compas.geometry import Transformation
from compas_fab.backends import RosClient
from roslibpy import Topic
import urllib.request

from aruco_markers import *

import queue
import threading

# bufferless VideoCapture
class VideoCapture:

    def __init__(self, name):
        self.cap = cv2.VideoCapture(name)
        self.lock = threading.Lock()
        self.tick=0
        self.buffer = None
        t = threading.Thread(target=self._reader)
        t.daemon = True
        t.start()

  # read frames as soon as they are available, keeping only most recent one
    def _reader(self):
        while True:
            print("Reading frame after %i"%self.tick)
            start = time.time()
            ret, frame = self.cap.read()
            print("Frame %i arrived after %.2f"% (self.tick+1, (time.time() - start)))
            self.lock.acquire()
            if self.buffer is not None:
                print ("Frame %i Discarded" % self.tick)
            self.buffer = frame
            self.lock.release()
            self.tick +=1

    def read(self):
        while self.buffer is None:
            pass
        self.lock.acquire()
        frame = self.buffer.copy()
        self.buffer = None # Remove the frame from buffer
        self.lock.release()
        return frame

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Camera calibration')
    parser.add_argument('--url', type=str, required=True, help='url of the camera stream')
    parser.add_argument('--calibration_file', type=str, required=True, help='YML file to load calibration matrices')

    parser.add_argument('--dictionary_name', type=str, default="DICT_4X4_50", help='Name of aruco dictionary, default DICT_4X4_50')
    parser.add_argument('--markers_count_x', type=int, default=4, help='Name of aruco dictionary, default DICT_4X4_50')
    parser.add_argument('--markers_count_y', type=int, default=2, help='Name of aruco dictionary, default DICT_4X4_50')
    parser.add_argument('--marker_size', type=float, required=True, help='Real size of the printed marker (convention is mm)')
    parser.add_argument('--marker_spacing', type=float, required=True, help='Spacing between markers (convention is mm)')
    parser.add_argument('--ros_ip', type=str, default='localhost', required=False, help='IP address of the ROS network')

    parser.add_argument('--output_board', action='store_true',  help='If set, a png image of the ArUco Board will be saved.')

    args = parser.parse_args()

    client = RosClient(args.ros_ip)
    client.run()
    print ("ROS Connected")

    ros_topic = Topic(client, '/camera_frames', 'std_msgs/String')
    ros_topic.advertise()
    print ("Topic Advertised")

    K, D = load_coefficients(args.calibration_file)

    # Generate and save image of board
    dictionary_name = args.dictionary_name
    markers_count_x, markers_count_y = args.markers_count_x, args.markers_count_y # 4x3 [Toolchanger Docker], 3x1 [Clamp to Joint]

    board = load_aruco_board(args.dictionary_name, args.markers_count_x, args.markers_count_y, args.marker_size, args.marker_spacing)

    if args.output_board:
        resolution_per_mm = 20
        save_aruco_board(board, args.marker_size, args.marker_spacing, resolution_per_mm)

    def put_text(frame, text, org):
        frame = cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,0), 2, cv2.LINE_AA)
        return cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 1, cv2.LINE_AA)

    # try:
        # cv2 Video Capture does deal with the multipart jpeg stream.
    vcap = VideoCapture(args.url)
    retval = 0
    while True:
        start = time.time()
        frame = vcap.read() # Reads one JPEG of the M-JPEG stream

        # * Open CV style quit mechanism
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            break

        ids, corners, rejected_img_points = detect_markers(frame, board.dictionary)

        # *See *Refine marker detection* in https://docs.opencv.org/3.4.15/db/da9/tutorial_aruco_board_detection.html
        cv2.aruco.refineDetectedMarkers(frame, board, corners, ids, rejected_img_points)

        # * Check if any marker is detected
        if np.all(ids is not None):

            # * Draw individual marker square
            frame_with_markers = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)
            # * Estimate pose of Aruco Board
            retval, rvec, tvec = cv2.aruco.estimatePoseBoard(corners, ids, board, K, D, np.zeros((3,1)), np.zeros((3,1)))

            # * retval is the number of markers detected on the board
            if(retval):
                cv2.aruco.drawAxis(frame_with_markers, K, D, rvec, tvec, args.marker_size )
                # * Convert rvec and tvec to transformation and publish to ros_topic
                T = transformation_from_rvec_tvec(rvec, tvec)
                ros_topic.publish({'data': json.dumps(T.to_data())})

            # Change frame to the one drawn with markers
            frame = frame_with_markers


        resized_frame = cv2.resize(frame, (640,480), interpolation = cv2.INTER_AREA)
        put_text(resized_frame, '%i Markers' % retval, (20,440))
        fps = 1 / (time.time() - start)
        put_text(resized_frame, '%.1fFPS (Frame%i)' % (fps, vcap.tick), (20,40))
        cv2.imshow('VideoStream', resized_frame)

    # finally:
    #     if vcap.cap:
    #         vcap.cap.release()
        # pass

    cv2.destroyAllWindows()
    ros_topic.unadvertise()
    client.close()

# Launched with arguments:
# "args": ["--url", "http://192.168.1.2:80", "--calibration_file", "${workspaceFolder}/src//visual_docking/calibration_near_focus_800_600.yml", "--marker_size", "20.0", "--ros_ip", "192.168.1.4"]
