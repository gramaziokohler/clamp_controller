import datetime
import json
import logging
from collections import OrderedDict
from enum import Enum
from functools import partial
from threading import Thread
from types import SimpleNamespace
from typing import Dict, List, Optional, Tuple
import time

from clamp_controller.RemoteClampFunctionCall import RemoteClampFunctionCall
from compas.utilities import DataDecoder
from compas_fab.backends.ros import RosClient
from compas_rrc import AbbClient
from integral_timber_joints.process import (Action, Movement,
                                            RobotClampAssemblyProcess)

from robot_clamp_controller.BackgroundCommand import *


from roslibpy import Ros
import compas_rrc as rrc
import os   # Save file location
import tempfile  # Save file location

logger_model = logging.getLogger("app.model")
logger_ros = logging.getLogger("app.ros")


class RunStatus(Enum):
    RUNNING = 0
    STEPPING_FORWARD = 1
    ERROR = 2
    STOPPED = 3


class RobotClampExecutionModel(object):

    def __init__(self):
        self.process: RobotClampAssemblyProcess = None

        # Ordered Dict for easy accessing the movements via move_id
        self.movements = OrderedDict()

        self.status_update_interval_high_ms: int = 150  # ms
        self.status_update_interval_low_ms: int = 2000  # ms

        self.logger = logging.getLogger("app.mdl")

        # Flag to indicate if an movement is active
        self.run_status: RunStatus = RunStatus.STOPPED
        self.run_thread: Thread = None
        self.ros_clamps: RemoteClampFunctionCall = None
        self.ros_robot: AbbClient = None
        self.operator_confirm = False

        self.current_action: Action = None  # Pointer to the currently selected action
        # Pointer to the currently selected movement
        self.current_movement: Movement = None

        # Settings
        self.settings = {}  # type: dict(str, str)
        self.load_settings()

    def load_settings(self, path=None):
        if path is None:
            path = os.path.join(tempfile.gettempdir(),
                                "itj_process_execution_setting.json")

        # Default Settings
        self.settings = {
            # Speed Settings in mm/s
            'speed.transfer.rapid': 500,
            'speed.transfer.caution': 100,
            'speed.transit.rapid': 800,
            'speed.toolchange.approach.withtool': 20,
            'speed.toolchange.approach.notool': 40,
            'speed.toolchange.approach.clamp_on_structure': 5,
            'speed.toolchange.retract.withtool': 20,
            'speed.toolchange.retract.notool': 40,
            'speed.toolchange.retract.clamp_on_structure': 5,
            'speed.assembly.inclamp': 50,   # Sliding beam into clamp.
            'speed.assembly.noclamp': 20,   # Simply putting it down
            'speed.assembly.clamping': 20,  # 
            'speed.gripper.approach': 50, # Approaching Pickup Station
            'speed.gripper.retract': 50,    # Retract after placing
            'speed.clamp.rapid': 5,
            'robot.joint_offset' : [0,0,0,0,0,0]
        }

        # Load Previously saved settings if exist
        if os.path.exists(path):
            with open(path, 'r') as f:
                self.settings.update(json.load(f))
            logger_model.info("Settings loaded from %s." % path)

        # Save it back to disk (useful during development)
        with open(path, 'w') as f:
            json.dump(self.settings, f, indent=4, sort_keys=True)
            logger_model.info("Settings saved to %s." % path)

    def load_process(self, json_path):
        with open(json_path, 'r') as f:
            self.process = json.load(f, cls=DataDecoder)
        logger_model.info("load_process(): %s" % self.process_description)

        # Organize movements into an OrderedDict collection for easier manupulation
        self.movements = OrderedDict()  # type : OrderedDict(Movement)
        for i, action in enumerate(self.process.actions):
            action.action_id = 'a_%i' % i
            for j, movement in enumerate(action.movements):
                move_id = "m%i_%i" % (i, j)
                movement.move_id = move_id
                self.movements[move_id] = movement


    def ros_clamps_callback(message, q=None):
        # ROS command comes from a separate thread.
        # To maintain single threaded access to the Radio / Clamp,
        # we convert the ROS Command to a BackgroundCommand and place it in background command queue

        message_type = message['instruction_type']

        logger_ros.info("Ros Message Received: %s" % message)
        if message_type == "CLAMPS_JAMMED":
            sequence_id = message['sequence_id']
            instructions = message['instruction_body']
            q.put(SimpleNamespace(type=BackgroundCommand.EXE_CLAMPS_JAMMED,
                                    clmap_pos_velo=instructions, sequence_id=sequence_id))

    def connect_ros_clamps(self, ip, q):
        """Function to connect to ROS CLamps Client.
        Returns True on successful connection"""

        # Disconnect from previous host
        if (self.ros_clamps is not None) and (self.ros_clamps.is_connected):
            try:
                self.ros_clamps.close()
                logger_model.info("Previous Clamps ROS host disconnected")
            except:
                pass

        self.ros_clamps = RemoteClampFunctionCall(
            ip, status_change_callback=partial(self.ros_clamps_callback, q=q))
        try:
            # This runs in a separate thread
            self.ros_clamps.run(timeout=2)
            time.sleep(0.5)
            logger_model.info("Clamps ROS host connected")
            return True
        except:
            self.ros_clamps = None
            return False
            pass

    def connect_ros_robots(self, ip, q):
        """Function to connect to ROS CLamps Client.
        Returns True on successful connection"""

        # Disconnect from previous host
        if (self.ros_robot is not None) and (self.ros_robot.ros.is_connected):
            try:
                self.ros_robot.close()
                logger_model.info("Previous ABB Robot ROS host disconnected")
            except:
                pass

        try:
            ros = RosClient(ip)
            # This runs in a separate thread
            ros.run()
            self.ros_robot = rrc.AbbClient(ros, '/robot_11')
            time.sleep(0.5)
            logger_model.info("ABB Robot ROS host connected")
            return True
        except:
            self.ros_robot = None
            return False
            pass

    def joint_offset(self, robot_joint_values):
        assert len(robot_joint_values) == 6
        return [(value + offset) for (value, offset) in zip(robot_joint_values, self.settings["robot.joint_offset"])]

    @property
    def process_description(self):
        if self.process is None:
            return "Process Not Loaded"
        else:
            return "Process Loaded (%i Beams, %i Actions, %i Movements)" % (
                len(self.process.assembly.sequence),
                len(self.process.actions),
                len(self.process.movements),
            )