import argparse

import cv2
import numpy as np

from visual_docking.aruco_markers import save_coefficients

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def calibrate(url, square_size, width=9, height=6):
    """ Apply camera calibration operation for images in the given directory path. """
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,6,0)
    objp = np.zeros((height*width, 3), np.float32)
    objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)

    objp = objp * square_size

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    try:
        vcap = cv2.VideoCapture(url)

        while True:
            _ret, frame = vcap.read()
            cv2.imshow('VideoStream', frame)

            key = cv2.waitKey(1)
            if key & 0xFF == ord('q'):
                break

            if key & 0xFF == ord('c'):
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                # Find the chess board corners
                ret, corners = cv2.findChessboardCorners(gray, (width, height), None)

                # If found, add object points, image points (after refining them)
                if ret:
                    print('Image captured, object points found')
                    objpoints.append(objp)

                    corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                    imgpoints.append(corners2)

                    # # Draw and display the corners
                    # frame = cv2.drawChessboardCorners(frame, (width, height), corners2, ret)
                else:
                    print('Image captured but NO object points found')


    finally:
        if vcap:
            vcap.release()

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    cv2.destroyAllWindows()

    return [ret, mtx, dist, rvecs, tvecs]


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Camera calibration')
    parser.add_argument('--url', type=str, required=True, help='url of the camera stream')
    parser.add_argument('--square_size', type=float, required=False, help='chessboard square size')
    parser.add_argument('--width', type=int, required=False, help='chessboard width size, default is 9')
    parser.add_argument('--height', type=int, required=False, help='chessboard height size, default is 6')
    parser.add_argument('--save_file', type=str, required=True, help='YML file to save calibration matrices')

    args = parser.parse_args()
    ret, mtx, dist, rvecs, tvecs = calibrate(args.url, args.square_size, args.width, args.height)
    save_coefficients(mtx, dist, args.save_file)
    print("Calibration is finished. RMS: ", ret)
