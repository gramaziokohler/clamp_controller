import cv2
import cv2.aruco
import numpy as np


def detect_markers(frame, aruco_dict = cv2.aruco.DICT_6X6_250):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.Dictionary_get(aruco_dict)
    parameters =  cv2.aruco.DetectorParameters_create()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR
    corners, ids, _rejected_img_points = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    frame_markers = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)

    return ids, corners, frame_markers


def estimate_pose(frame, marker_real_size, matrix_coefficients=None, distortion_coefficients=None):
    if matrix_coefficients is None:
        imsize = frame.shape
        matrix_coefficients = np.array([[ 2000.,    0., imsize[0]/2.],
                                        [    0., 2000., imsize[1]/2.],
                                        [    0.,    0.,           1.]])

    if distortion_coefficients is None:
        distortion_coefficients = np.zeros((5,1))

    ids, corners, _frame_markers = detect_markers(frame)
    rvecs, tvecs, marker_points = [], [], []
    estimated_frames = None

    if np.all(ids is not None):
        for i in range(len(ids)):
            # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
            rvec, tvec, marker_points = cv2.aruco.estimatePoseSingleMarkers(corners[i], marker_real_size, matrix_coefficients, distortion_coefficients)
            (rvec - tvec).any()  # get rid of that nasty numpy value array error

            rvecs.append(rvec)
            tvecs.append(tvec)

    return ids, corners, rvecs, tvecs, marker_points, estimated_frames


def save_coefficients(mtx, dist, path):
    """ Save the camera matrix and the distortion coefficients to given path/file. """
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    cv_file.write("K", mtx)
    cv_file.write("D", dist)
    cv_file.release()


def load_coefficients(path):
    """ Loads camera matrix and distortion coefficients. """
    # FILE_STORAGE_READ
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)

    # note we also have to specify the type to retrieve other wise we only get a
    # FileNode object back instead of a matrix
    camera_matrix = cv_file.getNode("K").mat()
    dist_matrix = cv_file.getNode("D").mat()

    cv_file.release()
    return [camera_matrix, dist_matrix]
