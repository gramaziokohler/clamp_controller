# 70mm Viewing Distance Validation

## Calibration

Manual focus to about 70mm

Calibration with grey checkerboard 4mm (8x6)

ArUco board parameters:

```
"--marker_size", "10.0",
"--marker_spacing", "2.0",
"--markers_count_x", "4",
"--markers_count_y", "2",
```



## Tuning

Some effort is spent to tune the pose detection to be stable and return accurate result in the expense of frame rate and . Using instructions from [ArUco Library Documentation](https://docs.google.com/document/d/1QU9KoBtjSM2kF6ITOjQ76xqL7H0TEtXriJX5kwi9Kgc/edit?usp=sharing) and [Tutorial: Detection of ArUco Markers](https://docs.opencv.org/3.4.15/d5/dae/tutorial_aruco_detection.html) . Below is an excerpt of the detection parameters.

```
def detect_markers(frame, aruco_dict = None):
    if aruco_dict is None:
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    parameters =  cv2.aruco.DetectorParameters_create()
    # Tunning for improving accuracy
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR
    # Tunning for improving performance under assumption that marker covers 10% of max dimension. (min rate 0.4)
    parameters.minMarkerPerimeterRate  = 0.3 # Margin given from 0.4 # default = 0.03

    if max(frame.shape[0:2]) > 1280:
        # Tunning for 1280x1024 resolution or above
        parameters.adaptiveThreshWinSizeMin = 5 # default = 3
        parameters.adaptiveThreshWinSizeMax = 45 # default = 23
        # Tunning for better pose estimation accuracy
        parameters.minDistanceToBorder = 40 # default = 3
    else:
        # Tunning for better pose estimation accuracy
        parameters.minDistanceToBorder = 20 # default = 3

    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    return ids, corners, rejected_img_points
```



 

## Validation

Detection **frame rate** monitored by `rostopic hz camera_frames -w 10` is around 3 to 3.5 Hz.

**Deviation** is observed in Rhino GH using the transformation streamed via ROS.

Translational Deviation in the XY plane (of marker board) detection is pretty accurate. Mostly in the range of 0.5mm.

Deviation will induce some tilt error. This is not ideal.

Rotation does not induce much tilt error

## Conclusion

The detection parameters calibrated as of now is pretty robust and stable. Repeatability is high for going back to a ground-zero position.

However, the obtainable transformation from a camera pose going to the ground-zero pose is not precise. This is probably due to camera distortion or camera calibration limitation caused by the relatively short focal length. It is also possible that some error is introduced by objects near the edge of the lens going out of focus.

I **recommend a two step correction routine** in the production controller, allowing first a rough correction, and a second round of refinement.



 