import compas_rrc as rrc
from compas.geometry import Frame, Point, Vector
from compas_fab.backends.ros import RosClient
import time
from datetime import datetime
from clamp_controller.RemoteClampFunctionCall import RemoteClampFunctionCall
from clamp_controller.CommanderGUI import ROS_VEL_GOTO_COMMAND
from clamp_controller.RosCommand import *

import logging

def current_milli_time(): return int(round(time.time() * 1000))

logger_exe = logging.getLogger('test')
logger_exe.setLevel(logging.DEBUG)
# create console handler and set level to debug
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
# create formatter
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
# add formatter to ch
ch.setFormatter(formatter)
# add ch to logger
logger_exe.addHandler(ch)

"""
This test is to check the stall feedback mechanism for the Clamp Controller:
- A RemoteClampFunctionCall object is used to maintain the connection to the ClampController
- A ROS sync move command is sent to one clamp
- During movement, the test script will monitor the status of the clamps using RemoteClampFunctionCall

- During movement, I will unplug the power of the Clamp to simulate a loss of connection.
- During movement, I will unplug the encoder cable of the Clamp to simulate a stall.
- During movement, I will quit the Clamp Controller Application

In all situations, the monitoring script need to raise a error message fast enough to indicate the stall of the clamp.

"""


def interpolate(start, end, total_steps, current_step):
    if current_step == 0:
        return start
    elif current_step == total_steps:
        return end
    elif (current_step > 0) & (current_step < total_steps):
        return [((e - s)/total_steps*current_step + s) for s, e in zip(start, end)]
    else:
        raise IndexError("current_step must be within 0 to total_steps")

CLAMP_START_RUNNING_TIMEOUT = 0.7
CLAMP_CONTROLLER_ALIVE_TIMEOUT = 3.5

if __name__ == '__main__':

    ip = "192.168.0.120"
    ros_clamps = RemoteClampFunctionCall(ip)
    clamps_id = ['s1']
    try:
        # This runs in a separate thread
        ros_clamps.run(timeout=2)
        time.sleep(0.5)
        logger_exe.info("Clamps ROS host connected. ip= %s" % ip)
    except:
        logger_exe.info("Failed to connect to Clamps ROS host. ip= %s" % ip)

        ros_clamps = None

    # * Send movement to Clamp Controller, wait for ROS controller to ACK
    logger_exe.info("Sending send")
    command_sent_time = time.time()
    velocity = 0.8
    position = 10
    power_percentage=80
    allowable_target_deviation=5
    clamps_pos_velo = [(clamp_id, position, velocity) for clamp_id in clamps_id]
    clamp_command = ROS_VEL_GOTO_COMMAND(clamps_pos_velo, power_percentage, allowable_target_deviation)
    message = ros_clamps.send(clamp_command)
    clamp_command_ack_timeout = 0.5 # ! Timeout for Clamp Controller to ACK command

    # Wait for ACK
    while (True):
        if message.acked == True:
            logger_exe.info("send ACK received.")
            break
        time_since = time.time() - command_sent_time
        # logger.debug(time_since)
        if time_since > clamp_command_ack_timeout:
            logger_exe.info("ACK not received from clamp controller within %s sec" % clamp_command_ack_timeout)
            exit(-1)
        time.sleep(0.02)


    # Track progress
    while(True):
        # * Monitor if the Clamp have started running
        if clamp_command.status != ROS_COMMAND.RUNNING:
            if current_milli_time() - message.send_time > CLAMP_START_RUNNING_TIMEOUT * 1000:
                logger_exe.warn("Sync Lost: Clamp status did not start running within %s sec" % CLAMP_START_RUNNING_TIMEOUT)
                exit(-1)

        # * Monitor if the Clamp Controller is still alive
        if clamp_command.status == ROS_COMMAND.RUNNING:
            if ros_clamps.last_received_message_age_ms > CLAMP_CONTROLLER_ALIVE_TIMEOUT * 1000:
                logger_exe.warn("Sync Lost: Clamp Commander not alive for more than %s sec" % CLAMP_CONTROLLER_ALIVE_TIMEOUT)
                exit(-1)

        # * Monitor if the Clamp Controller reports a failed sync move.
        if clamp_command.status == ROS_COMMAND.FAILED:
            logger_exe.warn("Sync Lost: Clamp Command Failed.")
            exit(-1)

        # * Monitor if the Clamp Controller reports a failed sync move.
        if clamp_command.status == ROS_COMMAND.SUCCEED:
            logger_exe.warn("Clamp Move succeeded.")
            exit(-1)

        time.sleep(0.05)





