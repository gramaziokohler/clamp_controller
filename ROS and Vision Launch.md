## Launching ROS

Launch the following in a terminal window

```
cd ~/catkin_ws
source devel/setup.bash
roslaunch main.launch
```

If launching only the ros_bridge:

```
roslaunch rosbridge_server rosbridge_websocket.launch
```

### Modifying launch file

To edit the launch file:

```
nano main.launch
```

## Updating compas_rrc Driver

To pull latest `compas_rrc_ros` repo:

```
cd ~/catkin_ws/src/compas_rrc_ros
git pull
```

## Launching Camera Marker Tracker

Use the following lines to open each of the marker tracking software: 

```
conda activate opencv

python ~/clamp_controller/src/visual_docking/aruco_board_pose_from_network.py --url http://192.168.1.100 --calibration_file /home/itj/clamp_controller/calibrations/cam100_1600x1200.yml --marker_size 10.0 --marker_spacing 2.0 --markers_count_x 4 --markers_count_y 2 --ros_ip 192.168.1.2 --ros_topic TC4_Camera
```

Calibration

```
conda activate opencv
python ~/clamp_controller/src/visual_docking/calibration.py --url http://192.168.1.100 --square_size 3.0 --width 9 --height 6 --save_file cam100_1600x1200.yml
```

## Monitoring

Use this to monitor connected ROS clients:

```
rostopic echo connected_clients
```

Use this to monitor tracked marker frames: 

```
rostopic hz camera_frames -w 10
rostopic echo TC4_Camera
```

## IP Address

IP Address of this machine (in camera network): `192.168.1.2`

- Camera on tool changer `192.168.1.100`
- Camera on `c1`:  `192.168.1.101`

IP Address of ROS core (in RFL network): `192.168.0.120`

- IP Address of RFL robot 11: `192.168.0.11` (namespace = `/rob1`)
- IP Address of RFL robot 12: `192.168.0.12`

## Note

The IP Address of this ROS server is 192.168.1.2 

Use a LAN cable to connect this computer Ethernet (eno1) to the `itj` router. 

