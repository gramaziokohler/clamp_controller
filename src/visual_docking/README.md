# OpenCV hand-eye calibration


## Setup
Raspberry Pi Zero
Camera V2.1
https://chev.me/arucogen/



## Installation

    conda create -n opencv opencv compas_fab python=3.8
    conda activate opencv
    pip install opencv-contrib-python

## Network
    Router ssid = itj
    Router pwd = integral_timber_joints
    Router configured to fix ip of  B8:27:EB:F4:8D:FB  at  192.168.1.102

## Calibration

    python calibration.py --url tcp://192.168.1.102:5000 --square_size 12.843 --save_file calibration_1280x720.yml --width 9 --height 6

## Usage

    python pose_from_network.py --url tcp://192.168.1.102:5000 --calibration_file calibration_1280x720.yml --marker_size 43.2 --ros_ip 192.168.1.2

    conda activate opencv
    cd ..\clamp_controller\src\visual_docking\
    python aruco_board_pose_from_network.py --url http://192.168.1.100 --calibration_file calibration_1280x720.yml --marker_size 10.0 --marker_spacing 2.0 --ros_ip 192.168.1.2
