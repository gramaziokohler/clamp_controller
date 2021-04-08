import urllib.request

import cv2
import numpy as np

from .aruco_markers import estimate_pose

if __name__ == '__main__':
    import sys

    import matplotlib.pyplot as plt
    
    if len(sys.argv) == 1:
        print('usage: python pose_from_url.py url   (eg. http://192.168.3.124/capture)')
        sys.exit(-1)

    url = sys.argv[1]

    while True:
        resp = urllib.request.urlopen(url)
        image = np.asarray(bytearray(resp.read()), dtype='uint8')
        frame = cv2.imdecode(image, cv2.IMREAD_COLOR)

        cv2.imshow('ImageStream', frame)

        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            break
        
        if key & 0xFF == ord('c'):
            ids, corners, rvecs, tvecs, marker_points, estimated_frames = estimate_pose(frame)
            if np.all(ids is not None):
                print('Found {} markers'.format(len(ids)))
                for i in range(len(ids)):
                    print(ids[i], rvecs[i], tvecs[i], corners[i].tolist())
    
    cv2.destroyAllWindows()

    if np.all(ids is not None):
        plt.figure()
        plt.imshow(estimated_frames)
        plt.show()
