# OpenCV hand-eye calibration


## Setup
Raspberry Pi Zero
Camera V2.1
https://chev.me/arucogen/



## Installation

    conda create -n opencv opencv compas_fab python=3.8
    conda activate opencv
    pip install opencv-contrib-python

## Camera Network
    Router ssid = itj
    Router pwd = integral_timber_joints
    Camera on Toolchanger IP 192.168.1.100


## Calibration (Camera Intrinsic after changing focus)

    conda activate opencv

    python -m visual_docking.calibration --url http://192.168.1.100 --square_size 3 --width 9 --height 6 --save_file cam100_1600x1200.yml

    python -m visual_docking.calibration --url http://192.168.1.101 --square_size 5 --width 8 --height 6 --save_file cam101_1600x1200.yml

    python -m visual_docking.calibration --url http://192.168.1.102 --square_size 5 --width 8 --height 6 --save_file cam102_1600x1200.yml

    python -m visual_docking.calibration --url http://192.168.1.103 --square_size 5 --width 8 --height 6 --save_file cam103_1600x1200.yml

    python -m visual_docking.calibration --url http://192.168.1.104 --square_size 5 --width 8 --height 6 --save_file cam104_1600x1200.yml

## Calibration (Marker Position on device)

1. Use the grasshopper file `marker_pose_and_calibration.gh` in `clamp_hardware repo.
2. Model and reference the robot flange frame, and marker frame.
3. Run `aruco_board_pose_from_network.py` similar to the code below.
4. Save the `*_t_flange_from_marker.json` and `*_t_camera_from_marker.json` using the GH script.


## Stream marker transformation from OpenCV

    python pose_from_network.py --url tcp://192.168.1.102:5000 --calibration_file calibration_1280x720.yml --marker_size 43.2 --ros_ip 192.168.1.2

    conda activate opencv

    python -m visual_docking.aruco_board_pose_from_network --ros_ip 192.168.1.2  --url http://192.168.1.100 --calibration_file ~/clamp_controller/calibrations/cam100_1600x1200.yml --marker_size 10.0 --marker_spacing 2.0 --markers_count_x 4 --markers_count_y 2 --ros_topic TC4_Camera
    
    python -m visual_docking.aruco_board_pose_from_network --ros_ip 192.168.1.2  --url http://192.168.1.101 --calibration_file ~/clamp_controller/calibrations/cam101_1600x1200.yml --marker_size 25.0 --marker_spacing 5.0 --markers_count_x 3 --markers_count_y 1 --ros_topic c1

    python -m visual_docking.aruco_board_pose_from_network --ros_ip 192.168.1.2  --url http://192.168.1.102 --calibration_file ~/clamp_controller/calibrations/cam102_1600x1200.yml --marker_size 25.0 --marker_spacing 5.0 --markers_count_x 3 --markers_count_y 1 --ros_topic c2

    python -m visual_docking.aruco_board_pose_from_network --ros_ip 192.168.1.2  --url http://192.168.1.103 --calibration_file ~/clamp_controller/calibrations/cam103_1600x1200.yml --marker_size 25.0 --marker_spacing 5.0 --markers_count_x 3 --markers_count_y 1 --ros_topic c3

    python -m visual_docking.aruco_board_pose_from_network --ros_ip 192.168.1.2  --url http://192.168.1.104 --calibration_file ~/clamp_controller/calibrations/cam104_1600x1200.yml --marker_size 25.0 --marker_spacing 5.0 --markers_count_x 3 --markers_count_y 1 --ros_topic c4

