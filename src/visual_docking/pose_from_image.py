import cv2

from .aruco_markers import estimate_pose


def get_frame_from_file(filename):
    frame = cv2.imread(filename)
    return frame


if __name__ == '__main__':
    import sys

    import matplotlib.pyplot as plt
    
    if len(sys.argv) == 1:
        print('usage: python pose_from_image.py filename')
        sys.exit(-1)

    filename = sys.argv[1]
    frame = get_frame_from_file(filename)

    plt.figure()

    ids, corners, rvecs, tvecs, marker_points, estimated_frames = estimate_pose(frame)
    plt.imshow(estimated_frames)
    plt.show()
    for i in range(len(ids)):
        print(ids[i], rvecs[i], tvecs[i], corners[i].tolist())
