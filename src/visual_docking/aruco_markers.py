import cv2
import numpy as np
from compas.geometry import Transformation


def detect_markers(frame, aruco_dict=None):
    if aruco_dict is None:
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

    parameters = cv2.aruco.DetectorParameters_create()
    # Tunning for improving accuracy
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR
    # Tunning for improving performance under assumption that marker covers 10% of max dimension. (min rate 0.4)
    parameters.minMarkerPerimeterRate = 0.3  # Margin given from 0.4 # default = 0.03

    if max(frame.shape[0:2]) > 1280:
        # Tunning for 1280x1024 resolution or above
        parameters.adaptiveThreshWinSizeMin = 5  # default = 3
        parameters.adaptiveThreshWinSizeMax = 45  # default = 23
        # Tunning for better pose estimation accuracy
        parameters.minDistanceToBorder = 40  # default = 3
    else:
        # Tunning for better pose estimation accuracy
        parameters.minDistanceToBorder = 20  # default = 3

    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    return ids, corners, rejected_img_points


def estimate_pose(frame, marker_real_size, matrix_coefficients=None, distortion_coefficients=None, aruco_dict=None):
    if matrix_coefficients is None:
        imsize = frame.shape
        matrix_coefficients = np.array([[2000.,    0., imsize[0]/2.],
                                        [0., 2000., imsize[1]/2.],
                                        [0.,    0.,           1.]])

    if distortion_coefficients is None:
        distortion_coefficients = np.zeros((5, 1))

    ids, corners, rejected_img_points = detect_markers(frame, aruco_dict)
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


def transformation_from_rvec_tvec(rvec, tvec):

    R, _jacobian = cv2.Rodrigues(rvec)

    r1, r2, r3 = R.tolist()
    r1.append(tvec.flat[0] * 1)
    r2.append(tvec.flat[1] * 1)
    r3.append(tvec.flat[2] * 1)
    T_matrix = [r1, r2, r3, [0, 0, 0, 1]]
    return Transformation.from_matrix(T_matrix)


def load_aruco_board(dictionary_name, markers_count_x, markers_count_y, marker_size, marker_spacing):

    dictionary_id = eval("cv2.aruco." + dictionary_name)

    board = cv2.aruco.GridBoard_create(markers_count_x, markers_count_y, marker_size, marker_spacing, cv2.aruco.Dictionary_get(dictionary_id))
    return board


def save_aruco_board(board, marker_size, marker_spacing, resolution_per_mm):
    """Save png image of aruco board"""
    board_name = board.dictionary_name + "_%ix%i_%i_%i" % (board.markers_count_x, board.markers_count_y, )
    s = resolution_per_mm * marker_size
    p = resolution_per_mm * marker_spacing
    board_px_size = (int(board.markers_count_x * (s + p) + p), int(board.markers_count_y * (s + p) + p))
    img = board.draw(board_px_size, marginSize=int(p))
    cv2.imwrite(board_name + ".png", img, [cv2.IMWRITE_PNG_COMPRESSION, 0])
    cv2.imshow(board_name, img)


def save_coefficients(mtx, dist, path):
    """ Save the camera matrix and the distortion coefficients to given path/file. """
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    cv_file.write("K", mtx)
    cv_file.write("D", dist)
    cv_file.release()


def load_coefficients(path):
    """ Loads camera matrix and distortion coefficients. """
    # FILE_STORAGE_READ
    import os
    assert os.path.exists(path)
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)

    # note we also have to specify the type to retrieve other wise we only get a
    # FileNode object back instead of a matrix
    camera_matrix = cv_file.getNode("K").mat()
    dist_matrix = cv_file.getNode("D").mat()

    cv_file.release()
    return [camera_matrix, dist_matrix]
