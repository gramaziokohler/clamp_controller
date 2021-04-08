# OpenCV hand-eye calibration

## Installation

    conda create -n opencv opencv compas_fab python=3.8
    conda activate opencv

## Calibration

    python calibration.py --url tcp://192.168.95.70:5000 --square_size 12.843 --save_file calibration_1280x720.yml --width 9 --height 6

## Usage

    python pose_from_network.py --url tcp://192.168.95.70:5000 --calibration_file calibration_1280x720.yml --marker_size 43.2
