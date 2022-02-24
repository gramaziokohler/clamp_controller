import compas_rrc as rrc
from compas.geometry import Frame, Point, Vector
from compas_fab.backends.ros import RosClient
import time
from datetime import datetime
from clamp_controller.RemoteClampFunctionCall import RemoteClampFunctionCall
from clamp_controller.CommanderGUI import ROS_VEL_GOTO_COMMAND

import logging


logger = logging.getLogger('test')
logger.setLevel(logging.DEBUG)
# create console handler and set level to debug
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
# create formatter
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
# add formatter to ch
ch.setFormatter(formatter)
# add ch to logger
logger.addHandler(ch)

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


if __name__ == '__main__':

    ip = "192.168.0.120"
    ros_clamps = RemoteClampFunctionCall(ip)
    clamps_id = ['s1']
    try:
        # This runs in a separate thread
        ros_clamps.run(timeout=2)
        time.sleep(0.5)
        logger.info("Clamps ROS host connected. ip= %s" % ip)
    except:
        logger.info("Failed to connect to Clamps ROS host. ip= %s" % ip)

        ros_clamps = None

    # * Send movement to Clamp Controller, wait for ROS controller to ACK
    logger.info("Sending send")
    command_sent_time = time.time()
    velocity = 0.8
    position = 10
    power_percentage=80
    allowable_target_deviation=5
    clamps_pos_velo = [(clamp_id, position, velocity) for clamp_id in clamps_id]
    command = ROS_VEL_GOTO_COMMAND(clamps_pos_velo, power_percentage, allowable_target_deviation)
    message = ros_clamps.send(command)
    clamp_command_ack_timeout = 0.5 # ! Timeout for Clamp Controller to ACK command
    # Wait for ACK
    while (True):
        if message.done == True:
            logger.info("send ACK received.")
            break
        time_since = time.time() - command_sent_time
        # logger.debug(time_since)
        if time_since > clamp_command_ack_timeout:
            logger.info("ACK not received from clamp controller within %s sec" % clamp_command_ack_timeout)
            exit(-1)
        time.sleep(0.02)

    # * Wait for first status update - (Should come in within 150ms)
    last_status_time = time.time()
    timeout = 0.2 # ! Timeout for first status to arrive
    while (True):
        if ros_clamps.last_status_time > last_status_time:
            logger.info("First Status arrived")
            logger.info("ros_clamps.last_command_success = %s" % ros_clamps.last_command_success)
            logger.info("ros_clamps.clamps_status = %s" % ros_clamps.clamps_status)
            break
        if time.time() - last_status_time > timeout:
            logger.info("Status did not arrive within %s sec" % timeout)
            exit(-1)

    # Track progress
    timeout = 0.50 # ! Status update age limit
    while(True):
        status_age = time.time() - ros_clamps.last_status_time
        logger.info("last_command_success = %s, sync_move_inaction = %s" % (ros_clamps.last_command_success, ros_clamps.sync_move_inaction))
        logger.info("status_age = %s" % status_age)
        for clamp_id in clamps_id:
            logger.info(ros_clamps.clamps_status[clamp_id])

        if status_age > timeout:
            logger.error("Sync Lost: Status age too old")
            exit(-1)
        if ros_clamps.last_command_success == True:
            logger.info("Sync Move Complete")
            exit(0)
        if ros_clamps.sync_move_inaction == False and ros_clamps.last_command_success == False:
            # if any([ros_clamps.clamps_status[clamp_id]['is_running'] == False for clamp_id in clamps_id]):
            logger.error("Sync Lost: Clamp Commander Report SyncMove not in action, Last command Failed")
            exit(-1)
        time.sleep(0.05)





