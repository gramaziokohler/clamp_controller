# clamp_controller
Python-based high-level controller to monitor and command a network of distributed clamps.

This repo is part of the [Robotic Assembled Timber Structures with Integral Timber Joints](https://github.com/gramaziokohler/integral_timber_joints) project. 

## Repo folder structure

**/src/clamp_controller** - Contains all the import-able class and functions.

**/src/controller_instances** - Contains instances of customized controllers for different projects.

## Design Goals

**ClampModel**

Digital twin of a clamp. Contains all the properties (e.g. step/mm conversion, soft limit, address) of a clamp and the telemetry reading.

Also contain functions to: 

- Decode received telemetry

- Conversion between linear position and step position.

**SerialCommander**

The main model of the application. Contains:

- Instance of all available `ClampModel`
- Instance of the `SerialPort` that connects to the USBRadioDongle
- Instance of the `RosClampCommandListener` that connects to a ROS core via roslibpy

Functions to:

- Accept high-level synchronized move command for multiple clamps.
- Create low level move, home, speed-set and stop command.
- Send low level command to clamp with resend-on-Nack option.

**RosClampCommandListener**

A class that maintains the connection with ROS via roslibpy and listens-for (and replies-to) commands received as a rostopic. 

**CommanderGUI**

A long function that create the tkinter UI for monitoring and control. This UI 

Alternatively, it is possible to run the `SerialCommander` directly without UI, all the connections and settings can be accessed by code.

Credits
-------------

This repository was created by Pok Yin Victor Leung <leung@arch.ethz.ch> [@yck011522 ](https://github.com/yck011522) at [@gramaziokohler](https://github.com/gramaziokohler)

