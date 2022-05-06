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

## Usage

    python pose_from_network.py --url tcp://192.168.1.102:5000 --calibration_file calibration_1280x720.yml --marker_size 43.2 --ros_ip 192.168.1.2
    
    conda activate opencv
    cd ..\clamp_controller\
    python src\visual_docking\aruco_board_pose_from_network.py --url http://192.168.1.100 --calibration_file src/visual_docking/calibrations/cam1_70mm_1600_1200.yml --marker_size 10.0 --marker_spacing 2.0 --ros_ip 192.168.1.2
    
    conda activate opencv
    cd ..\clamp_controller\
    python src\visual_docking\aruco_board_pose_from_network.py --url http://192.168.1.101 --calibration_file src/visual_docking/calibrations/cam1_70mm_1600_1200.yml --marker_size 10.0 --marker_spacing 2.0 --ros_ip 192.168.1.2
    
    conda activate opencv
    cd ..\clamp_controller\
    python src\visual_docking\aruco_board_pose_from_network.py --url http://192.168.1.102 --calibration_file src/visual_docking/calibrations/cam1_70mm_1600_1200.yml --marker_size 10.0 --marker_spacing 2.0 --ros_ip 192.168.1.2
    
    conda activate opencv
    cd ..\clamp_controller\
    python src\visual_docking\aruco_board_pose_from_network.py --url http://192.168.1.103 --calibration_file src/visual_docking/calibrations/cam1_70mm_1600_1200.yml --marker_size 10.0 --marker_spacing 2.0 --ros_ip 192.168.1.2
    
    conda activate opencv
    cd ..\clamp_controller\
    python src\visual_docking\aruco_board_pose_from_network.py --url http://192.168.1.104 --calibration_file src/visual_docking/calibrations/cam1_70mm_1600_1200.yml --marker_size 10.0 --marker_spacing 2.0 --ros_ip 192.168.1.2
