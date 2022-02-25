import datetime
import logging
import queue
import time
import tkinter as tk
from enum import Enum
from functools import partial
from threading import Thread
from tkinter import ttk
from types import SimpleNamespace
from typing import Dict, List, Optional, Tuple

from clamp_controller.ClampModel import ClampModel
from clamp_controller.ScrewdriverModel import ScrewdriverModel, g_status_dict
from clamp_controller.CommanderGUI import *
from clamp_controller.RosClampCommandListener import RosClampCommandListener
from clamp_controller.SerialCommander import RosSerialCommander
from clamp_controller.RemoteClampFunctionCall import RosMessage


def current_milli_time(): return int(round(time.time() * 1000))

# This UI implemented a Model-View-Controller pattern.

# Model is a single ROSSerialCommander() object
# View is the TKInter UI that is created by CommanderGUI.create_commander_gui()
# Controller for the UI is the tk.mainloop().
# Controller for background task is the background_thread() that runs on a separate thread.
# - The background thread implemented a time-share based multi-task execution.
# - It is separated to handle_ui_commands() and update_status()


# Initialize logger for the controller
logger_ctr = logging.getLogger("app.ctr")
logger_ros = logging.getLogger("app.ros")
logger_sync = logging.getLogger("app.sync")
# Initialize multi-tasking variables.
last_status_update_time = 0
MIN_ALLOWABLE_TARGET_DEVIATION = 0.1

def background_thread(guiref, commander: RosSerialCommander, q):
    # This is a sudo time-split multi-task with priority execution.
    # Tasks higher up in the list have higher priority.
    # Task must return True if it was executed and return False if it was not executed.
    logger = logging.getLogger("app.bg")
    logger.info("Background Thread Started")
    while True:
        if handle_background_commands(guiref, commander, q):
            continue

        # * Polls Status updates from device and updates UI
        update_status(guiref, commander)

        # * Monitoring active command
        active_command = commander.get_running_command()
        if active_command is not None:
            logger.info("Monitoring Active Command: %s" % active_command.__class__.__name__)
            if type(active_command) == ROS_VEL_GOTO_COMMAND:
                check_sync_move(commander, active_command)
            if type(active_command) == ROS_SCREWDRIVER_GRIPPER_COMMAND:
                check_screwdriver_gripper_move(commander, active_command)

            # * Send ros status update
            commander.send_command_status_to_ros(commander.last_ros_command)

    logging.getLogger("app.bg").info("Background Thread Stopped")


def get_checkbox_selected_clamps(guiref, commander: RosSerialCommander) -> List[ClampModel]:
    # Determine if the clamps are selected
    clamps_selected = []
    for clamp in commander.clamps.values():
        if (guiref['status'][clamp.receiver_address]['checkbox'].get()):
            clamps_selected.append(clamp)
    return clamps_selected


def handle_background_commands(guiref, commander: RosSerialCommander, q):
    """Try to pop a command from the command q.
    Returns True if a message is popped regardless if it is processed or not."""

    try:
        msg = None
        msg = q.get(timeout=0.1)
        if hasattr(msg, 'type'):
            # * Handelling UI_SERIAL_CONNECT
            if msg.type == ClampControllerBackgroundCommand.UI_SERIAL_CONNECT:
                logger_ctr.info("Command Received to Connect to %s" % msg.port)
                commander.connect(msg.port)
                return True

            # * Handelling UI_ROS_CONNECT
            if msg.type == ClampControllerBackgroundCommand.UI_ROS_CONNECT:
                logger_ctr.info("Command Received to Connect to ROS at %s" % msg.ip)
                # Disconnect from previous host
                if (commander.ros_client is not None) and (commander.ros_client.is_connected):
                    try:
                        commander.ros_client.close()
                        # commander.ros_client.terminate()
                        logger_ctr.info("Previous ROS host disconnected")
                    except:
                        pass
                # Connect to new ROS host
                guiref['ros']['ros_status'].set("Connecting to ROS")
                commander.ros_client = RosClampCommandListener(msg.ip, partial(put_ros_command_to_queue, q=q))
                try:
                    commander.ros_client.run(timeout=2)  # This starts a separate thread
                    guiref['ros']['ros_status'].set("Connected to ROS")
                    logger_ctr.info("Ros Connected")
                except:
                    guiref['ros']['ros_status'].set("Not Connected")
                    pass
                return True

            def _send_goto_command(devices, position):
                """ Instruct commander to send a goto command

                Beware this is not a syncronous command, because the clamp velocity might be different.
                However, we still implement a stop_clamps() in case of messaging failure
                """
                if len(devices) == 0:
                    logger_ctr.warning("No device is selected for the movement command.")
                    return

                successes = []
                processed_devices = []
                for device in devices:
                    success = commander.send_clamp_to_jaw_position(device, position)
                    successes.append(success)
                    processed_devices.append(device)
                    if not success:
                        commander.stop_clamps(processed_devices)
                logger_ctr.info("Movement command executed. position = %smm, devices = %s, result = %s" % (position, devices, successes))
                return True

            # * Handelling CMD_CLAMP_GOTO
            if msg.type == ClampControllerBackgroundCommand.CMD_CLAMP_GOTO:
                if not commander.is_connected:
                    logger_ctr.warning("Connect to Serial Radio first")
                    return True

                # Check for selected clamps
                clamps_to_communicate = get_checkbox_selected_clamps(guiref, commander)
                clamps_to_communicate = [clamp for clamp in clamps_to_communicate if clamp.__class__ == ClampModel]
                _send_goto_command(clamps_to_communicate, msg.position)
                return True

            # * Handelling CMD_SCREWDRIVER_GOTO
            if msg.type == ClampControllerBackgroundCommand.CMD_SCREWDRIVER_GOTO:
                if not commander.is_connected:
                    logger_ctr.warning("Connect to Serial Radio first")
                    return True

                # Check for selected clamps
                clamps_to_communicate = get_checkbox_selected_clamps(guiref, commander)
                clamps_to_communicate = [clamp for clamp in clamps_to_communicate if clamp.__class__ == ScrewdriverModel]
                _send_goto_command(clamps_to_communicate, msg.position)
                return True

            # * Handelling CMD_STOP
            if msg.type == ClampControllerBackgroundCommand.CMD_STOP:
                if not commander.is_connected:
                    logger_ctr.warning("Connect to Serial Radio first")
                    return True

                # * Fail last running ros command
                command = commander.get_running_command()
                commander.fail_last_running_command()

                # * Stop active clamps first
                if isinstance(command, RequireMonitoring):
                    active_tool_ids = command.get_monitoring_tool_ids()
                    active_clamps = [commander.get_clamp_by_process_tool_id(tool_id) for tool_id in active_tool_ids]
                    result = commander.stop_clamps(active_clamps)
                    logger_ctr.info("Stop command sent to active Devices %s, result = %s" % (active_tool_ids, result))

                # * Stop other clamps
                all_clamps = commander.clamps.values()
                selected_clamps = get_checkbox_selected_clamps(guiref, commander)
                clamps_to_communicate = [clamp for clamp in all_clamps if clamp in selected_clamps]
                result = commander.stop_clamps(clamps_to_communicate)
                logger_ctr.info("Stop command sent to Selected Devices %s, result = %s" % (clamps_to_communicate, result))

                return True

            # * Handelling CMD_HOME (Generic for both Screwdriver and Clamps)
            if msg.type == ClampControllerBackgroundCommand.CMD_HOME:
                if not commander.is_connected:
                    logger_ctr.warning("Connect to Serial Radio first")
                    return True
                # Instruct commander to send command
                clamps_to_communicate = get_checkbox_selected_clamps(guiref, commander)
                results = commander.home_clamps(clamps_to_communicate)
                logger_ctr.info("Home command sent to %s, results = %s" % (clamps_to_communicate, results))
                return True

            # * Handelling CMD_CLAMP_VELO
            if msg.type == ClampControllerBackgroundCommand.CMD_CLAMP_VELO:
                if not commander.is_connected:
                    logger_ctr.warning("Connect to Serial Radio first")
                    return True
                # Instruct commander to send command
                velocity = msg.velocity
                clamps_to_communicate = get_checkbox_selected_clamps(guiref, commander)
                clamps_to_communicate = [clamp for clamp in clamps_to_communicate if clamp.__class__ == ClampModel]
                results = commander.set_clamps_velocity(clamps_to_communicate, velocity)
                logger_ctr.info("Sending Velocity command (%s) to Clamps %s, results = %s" % (velocity, clamps_to_communicate, results))
                return True

            # * Handelling CMD_SCREWDRIVER_VELO
            if msg.type == ClampControllerBackgroundCommand.CMD_SCREWDRIVER_VELO:
                if not commander.is_connected:
                    logger_ctr.warning("Connect to Serial Radio first")
                    return True
                # Instruct commander to send command
                velocity = msg.velocity
                clamps_to_communicate = get_checkbox_selected_clamps(guiref, commander)
                clamps_to_communicate = [clamp for clamp in clamps_to_communicate if clamp.__class__ == ScrewdriverModel]
                results = commander.set_clamps_velocity(clamps_to_communicate, velocity)
                logger_ctr.info("Sending Velocity command (%s) to Screwdrivers %s, results = %s" % (velocity, clamps_to_communicate, results))
                return True

            # * Handelling CMD_SCREWDRIVER_GRIPPER
            if msg.type == ClampControllerBackgroundCommand.CMD_SCREWDRIVER_GRIPPER:
                if not commander.is_connected:
                    logger_ctr.warning("Connect to Serial Radio first")
                    return True
                # Instruct commander to send command
                extend = msg.extend
                devices_to_communicate = get_checkbox_selected_clamps(guiref, commander)
                devices_to_communicate = [clamp for clamp in devices_to_communicate if clamp.__class__ == ScrewdriverModel]
                results = commander.set_screwdriver_gripper(devices_to_communicate, extend)
                if extend:
                    logger_ctr.info("Pin Gripper Extend sent to Screwdrivers %s, results = %s" % (devices_to_communicate, results))
                else:
                    logger_ctr.info("Pin Gripper Retract sent to Screwdrivers %s, results = %s" % (devices_to_communicate, results))
                return True

            # * Handelling CMD_POWER
            if msg.type == ClampControllerBackgroundCommand.CMD_POWER:
                if not commander.is_connected:
                    logger_ctr.warning("Connect to Serial Radio first")
                    return True
                # Instruct commander to send command
                power = msg.power
                clamps_to_communicate = get_checkbox_selected_clamps(guiref, commander)
                results = commander.set_clamps_power(clamps_to_communicate, power)
                logger_ctr.info("Sending Power command (%s) to %s, results = %s" % (power, clamps_to_communicate, results))
                return True

            # Return True if q.get() didn't return empty
            logger_ctr.warning("Unknown Background Message: msg.type = %s" % msg.type)
            return True

        # * Handelling ROS Commands
        # * ***********************
        # * Handelling ROS_VEL_GOTO_COMMAND
        if type(msg) == ROS_VEL_GOTO_COMMAND:
            command = msg  # type: ROS_VEL_GOTO_COMMAND
            print(command.data)
            # Keep track if the command is completed successful.
            # This will be set back to true by the sync watcher
            commander.fail_last_running_command()
            commander.last_ros_command = command

            if not commander.is_connected:
                logger_ctr.warning("ROS_VEL_GOTO_COMMAND: Connect to Serial Radio first")
                commander.change_command_status_and_send_to_ros(command, ROS_COMMAND.FAILED)
                return True

            # Retrieve the list of tuples
            clamps_pos_velo = command.clamps_pos_velo
            if len(clamps_pos_velo) == 0:
                logger_ctr.warning("ROS_VEL_GOTO_COMMAND has no instructions")
                commander.change_command_status_and_send_to_ros(command, ROS_COMMAND.SUCCEED)
                return True

            # Send RUNNING status to ros
            commander.change_command_status_and_send_to_ros(command, ROS_COMMAND.RUNNING)
            logger_ctr.info("ROS_VEL_GOTO_COMMAND Command Start: %s" % (command.data))

            # Set Power
            if command.power_percentage is not None:
                power = command.power_percentage
                clamps_to_communicate = [commander.get_clamp_by_process_tool_id(tool_id) for tool_id, position, velocity in clamps_pos_velo]
                successes = commander.set_clamps_power(clamps_to_communicate, power)
                logger_ctr.info("Sending Power command (%s) to %s, successes = %s" % (power, clamps_to_communicate, successes))
                if not all(successes):
                    logger_ctr.warning("ROS_VEL_GOTO_COMMAND send power setting failed.")
                    commander.change_command_status_and_send_to_ros(command, ROS_COMMAND.FAILED)
                    return True

            # Replace the process_tool_id with the retrived ClampModel
            clamp_pos_velo_list = [(commander.get_clamp_by_process_tool_id(tool_id), position, velocity) for tool_id, position, velocity in clamps_pos_velo]
            success = commander.sync_clamps_move(clamp_pos_velo_list)

            if success:
                # Log
                logger_ctr.info("ROS_VEL_GOTO_COMMAND Command Success")
            else:
                # Log
                clamps = [commander.clamps[clamp_id] for clamp_id, position, velocity in clamps_pos_velo]
                positions = [position for clamp_id, position, velocity in clamps_pos_velo]
                logger_ctr.warning("ROS Command Fail: send_clamp_to_jaw_position(%s,%s) Fail" % (clamps, positions))
                commander.change_command_status_and_send_to_ros(command, ROS_COMMAND.FAILED)

            return True

        # * Handelling ROS_STOP_COMMAND
        elif type(msg) == ROS_STOP_COMMAND:
            command = msg  # type: ROS_STOP_COMMAND
            commander.fail_last_running_command()
            commander.last_ros_command = command
            commander.change_command_status_and_send_to_ros(command, ROS_COMMAND.RUNNING)
            logger_ctr.info("ROS_STOP_COMMAND Command Start: %s" % (command.data))

            if not commander.is_connected:
                logger_ctr.warning("ROS_STOP_COMMAND: Connect to Serial Radio first")
                return True
            # Get the clamp objects from commander
            clamps_to_communicate = [commander.get_clamp_by_process_tool_id(tool_id) for tool_id in command.tools_id]
            # Instruct commander to send command
            result = commander.stop_clamps(clamps_to_communicate)
            logger_ctr.info("ROS_STOP_COMMAND Executed: Stop command to %s, result = %s" % (clamps_to_communicate, result))
            if all(result):
                commander.change_command_status_and_send_to_ros(command, ROS_COMMAND.SUCCEED)

            return True

        elif type(msg) == ROS_STOP_ALL_COMMAND:
            command = msg  # type: ROS_STOP_ALL_COMMAND
            commander.fail_last_running_command()
            commander.last_ros_command = command
            commander.change_command_status_and_send_to_ros(command, ROS_COMMAND.RUNNING)
            logger_ctr.info("ROS_STOP_ALL_COMMAND Command Start")

            if not commander.is_connected:
                logger_ctr.warning("ROS_STOP_ALL_COMMAND: Connect to Serial Radio first")
                return True
            # Get the clamp objects from commander
            result = commander.stop_all_clamps()
            commander.change_command_status_and_send_to_ros(command, ROS_COMMAND.SUCCEED)

            return True

        # * Handelling ROS_SCREWDRIVER_GRIPPER_COMMAND
        elif type(msg) == ROS_SCREWDRIVER_GRIPPER_COMMAND:
            command = msg  # type: ROS_SCREWDRIVER_GRIPPER_COMMAND # Type Hint Helper
            commander.fail_last_running_command()
            commander.last_ros_command = command


            if not commander.is_connected:
                logger_ctr.warning("ROS_SCREWDRIVER_GRIPPER_COMMAND: Connect to Serial Radio first")
                commander.change_command_status_and_send_to_ros(command, ROS_COMMAND.FAILED)
                return True

            # Get the clamp objects from commander
            try:
                device_to_communicate = commander.get_clamp_by_process_tool_id(command.tool_id)
            except ValueError as err:
                logger_ctr.warning("ROS_SCREWDRIVER_GRIPPER_COMMAND incorrect: %s" % err)
                commander.change_command_status_and_send_to_ros(command, ROS_COMMAND.FAILED)
                return True
            if not device_to_communicate.__class__ == ScrewdriverModel:
                logger_ctr.warning("ROS_SCREWDRIVER_GRIPPER_COMMAND %s is not screwdriver: %s" % device_to_communicate)
                commander.change_command_status_and_send_to_ros(command, ROS_COMMAND.FAILED)
                return True

            # Instruct commander to send command
            commander.change_command_status_and_send_to_ros(command, ROS_COMMAND.RUNNING)
            logger_ctr.info("ROS_SCREWDRIVER_GRIPPER_COMMAND Command Start: %s" % (command.data))
            successes = commander.set_screwdriver_gripper([device_to_communicate], command.extend)

            if all(successes):
                # Log
                logger_ctr.info("ROS_SCREWDRIVER_GRIPPER_COMMAND Command Executed: device = %s, extend = %s" % (device_to_communicate, command.extend))
            else:
                # Log
                logger_ctr.warning("ROS_SCREWDRIVER_GRIPPER_COMMAND Command Fail: device = %s, extend = %s" % (device_to_communicate, command.extend))
                commander.change_command_status_and_send_to_ros(command, ROS_COMMAND.FAILED)

            # TODO monitoring function that checks if the gripper have finished.
            return True

        # * Handelling ROS_REQUEST_STATUSUPDATE
        elif type(msg) == ROS_REQUEST_STATUSUPDATE:
            command = msg  # type: ROS_REQUEST_STATUSUPDATE # Type Hint Helper

            # Instruct commander to send status update
            commander.change_command_status_and_send_to_ros(command, ROS_COMMAND.SUCCEED)
            logger_ctr.info("ROS_REQUEST_STATUSUPDATE Command Replied via ROS")

            return True

        # * Handelling UnKnown ROS
        elif type(msg) == ROS_COMMAND:
            command = msg  # type: ROS_REQUEST_STATUSUPDATE # Type Hint Helper

            # Instruct commander to send status update
            commander.change_command_status_and_send_to_ros(command, ROS_COMMAND.FAILED)
            logger_ctr.info("ROS_REQUEST_STATUSUPDATE Command Replied via ROS")

            return True
        else:
            logger_ctr.warning("Unknown Background Message: type(message) = %s" % type(msg))
            return True

    except queue.Empty:
        return False


def update_status(guiref, commander: RosSerialCommander):
    """Regularly calling the commander to poll clamps for a status.
    Updates the GUI labels upon a successful poll. ALso updates the last communicated time if unsuccessful.
    """
    global last_status_update_time
    if ((commander.serial_port is None) or (not commander.serial_port.isOpen())):
        return False
    if (last_status_update_time + commander.status_update_interval_ms <= current_milli_time()):
        last_status_update_time = current_milli_time()

        # * Request a status update from clamp commander
        if isinstance(commander.last_ros_command, RequireMonitoring):
            # * If a command is active, only the clamps in the command are polled.
            active_clamps = [commander.get_clamp_by_process_tool_id(tool_id) for tool_id in commander.last_ros_command.get_monitoring_tool_ids()]
            updated_clamps = commander.update_clamps_status(active_clamps, 1)
        elif any([clamp.isMotorRunning for clamp in commander.get_clamps()]):
            # *  Update clamps whose motors are running
            active_clamps = [clamp for clamp in commander.get_clamps() if clamp.isMotorRunning]
            updated_clamps = commander.update_clamps_status(active_clamps, 1)
        else:
            # * Update only clamps that are marked checked in the UI
            checked_clamps = get_checkbox_selected_clamps(guiref, commander)
            updated_clamps = commander.update_clamps_status(checked_clamps, 1)

        # * Set UI values (non updated clamps are also updated because last-update time is updated)
        for clamp in commander.clamps.values():
            if clamp.ishomed is not None:
                guiref['status'][clamp.receiver_address]['home'].set("Yes" if clamp.ishomed else "No")
                guiref['status'][clamp.receiver_address]['home_label'].config(fg="red" if not clamp.ishomed else "black")
            # Clamp Running Label
            if clamp.isMotorRunning is not None:
                if clamp.isMotorRunning:
                    if clamp.isDirectionExtend:
                        guiref['status'][clamp.receiver_address]['motion'].set("Extending")
                        guiref['status'][clamp.receiver_address]['motion_label'].config(fg="green")
                    else:
                        guiref['status'][clamp.receiver_address]['motion'].set("Retracting")
                        guiref['status'][clamp.receiver_address]['motion_label'].config(fg="green")
                else:
                    guiref['status'][clamp.receiver_address]['motion'].set("Stopped")
                    guiref['status'][clamp.receiver_address]['motion_label'].config(fg="black")
            # Jaw Position Label
            if clamp.currentJawPosition is not None:
                guiref['status'][clamp.receiver_address]['position'].set("%04.1fmm" % clamp.currentJawPosition)
            # Step Error with orange Label > abs(100 steps)
            if clamp._raw_currentPosition is not None:
                guiref['status'][clamp.receiver_address]['error'].set("%3i steps" % int(clamp._raw_currentPosition - clamp._raw_currentTarget))
                guiref['status'][clamp.receiver_address]['power_label'].config(fg="orange" if abs(clamp._raw_currentPosition - clamp._raw_currentTarget) > 100 else "black")
            # Motor Power percentage with Orange Label > 90%
            if clamp.currentMotorPowerPercentage is not None:
                guiref['status'][clamp.receiver_address]['power'].set("%3i%%" % clamp.currentMotorPowerPercentage)
                guiref['status'][clamp.receiver_address]['power_label'].config(fg="orange" if abs(clamp.currentMotorPowerPercentage) > 90 else "black")

            # Battery percentage with Red Label < 10%
            if clamp.batteryPercentage is not None:
                guiref['status'][clamp.receiver_address]['battery'].set("%2i%%" % clamp.batteryPercentage)
                guiref['status'][clamp.receiver_address]['battery_label'].config(fg="red" if clamp.batteryPercentage < 10 else "black")

            # Clamp Last Communicate Time with Read Label > 500ms
            if clamp._state_timestamp is not None:
                guiref['status'][clamp.receiver_address]['last_com'].set("%2dms" % (current_milli_time() - clamp._state_timestamp))
                guiref['status'][clamp.receiver_address]['last_com_label'].config(fg="red" if (current_milli_time() - clamp._state_timestamp) > 500 else "black")
            if clamp._last_set_position is not None:
                guiref['status'][clamp.receiver_address]['last_pos'].set("%04.1fmm" % clamp._last_set_position)
            if clamp._last_set_velocity is not None:
                guiref['status'][clamp.receiver_address]['last_vel'].set("%3.2fmm/s" % clamp._last_set_velocity)

            # * Update Screwdriver specific values
            if isinstance(clamp, ScrewdriverModel):
                if clamp.currentGripperPosition != (None, None):
                    guiref['status'][clamp.receiver_address]['grip'].set("%3.1f / %3.1f mm" % clamp.currentGripperPosition)

                if clamp._raw_gripper_status is not None:
                    guiref['status'][clamp.receiver_address]['g_status'].set(g_status_dict[clamp._raw_gripper_status])
                    if clamp.gripper_is_moving:
                        guiref['status'][clamp.receiver_address]['g_status_label'].config(fg="green")
                    elif clamp.gripper_move_failed:
                        guiref['status'][clamp.receiver_address]['g_status_label'].config(fg="red")
                    else:
                        guiref['status'][clamp.receiver_address]['g_status_label'].config(fg="black")

        return True
    else:
        return False


def check_sync_move(commander: RosSerialCommander, command: ROS_VEL_GOTO_COMMAND) -> bool:
    """ This function checks all the clamps in ROS_VEL_GOTO_COMMAND.get_monitoring_tool_ids()
    The following check is performed:
    - If any clamp in the list have not reached its target (within threshold) and have stopped moveing.
    - Maybe other checkes will be added in the future.
    This function will request the commander to stop all clamps involved.

    Returns True if a check is performed and the command is still running.
    """
    if command.status != ROS_COMMAND.RUNNING:
        return False

    active_clamp_status_timeout_ms = 3500
    clamp_ids = command.get_monitoring_tool_ids()
    clamp_pos_velo = [(commander.get_clamp_by_process_tool_id(clamp_id), p, v) for (clamp_id, p, v) in command.clamps_pos_velo]

    def fail_routine(failed_clamp_id):
        # Send stop command to all acive clamps, messaging the failed_clamp_id last
        clamps_to_stop = [clamp_id for clamp_id in clamp_ids if clamp_id is not clamp]
        clamps_to_stop.append(failed_clamp_id)
        logger_sync.warning("check_sync_move failed. Stopping all clamps: %s" % clamps_to_stop)
        # Try to message them until they are all successful, max try 5 times
        for _ in range(5):
            successes = commander.stop_clamps(clamps_to_stop)
            if all(successes):
                break
        commander.change_command_status_and_send_to_ros(command, ROS_COMMAND.FAILED)

    # * Check every clamp that should be moving
    for clamp, target_jaw_position, velo in clamp_pos_velo:
        # * Compute target success based on allowable_target_deviation
        target_deviation = abs(target_jaw_position - clamp.currentJawPosition)
        allowable_deviation = max(abs(command.allowable_target_deviation), MIN_ALLOWABLE_TARGET_DEVIATION)
        target_reached = target_deviation < allowable_deviation
        if clamp.isMotorRunning == False and target_reached == False:
            logger_sync.warning("Sync Move Check Failed: %s stopped at %0.1fmm before reaching target %0.1fmm. Devaition %0.1fmm > %0.1fmm Initializing stop all clamps." %
                (clamp, clamp.currentJawPosition, target_jaw_position, target_deviation, allowable_deviation)
                )
            # Stop all clamps involved in the sync move
            fail_routine(clamp)
            return False
        if current_milli_time() - clamp._state_timestamp > active_clamp_status_timeout_ms:
            logger_sync.warning("Sync Move Failed: %s lost contact (timeout = %s ms)" % (clamp, active_clamp_status_timeout_ms))
            # Stop all clamps involved in the sync move
            fail_routine(clamp)
            return False

    # * Success if all clamps reached target or stopped
    if not any([clamp.isMotorRunning for clamp, _, _ in clamp_pos_velo]):
        logger_sync.info("Sync Move Monitoring Completed. All clamps stopped.")
        for clamp, target_jaw_position, _ in clamp_pos_velo:
            logger_sync.info("Sync Move Complete - %s current_pos = %0.1fmm (target = %0.1fmm)" % (clamp.process_tool_id, clamp.currentJawPosition, target_jaw_position))
        commander.change_command_status_and_send_to_ros(command, ROS_COMMAND.SUCCEED)
        return False

    # * Return True if the command is still running
    return True


def check_screwdriver_gripper_move(commander: RosSerialCommander, command: ROS_SCREWDRIVER_GRIPPER_COMMAND):
    """ This function check the clamp in ROS_SCREWDRIVER_GRIPPER_COMMAND.get_monitoring_tool_ids()
    The following check is performed:
    - If the gripper status (Extended or Retracted) match the
    This command is not a syncronized move, so there is no syncronized stop when the gripper fails.
    """
    if command.status != ROS_COMMAND.RUNNING:
        return False

    active_clamp_status_timeout_ms = 3500
    screwdriver_id = command.get_monitoring_tool_ids()[0]
    screwdriver = commander.get_clamp_by_process_tool_id(screwdriver_id)  # type: ScrewdriverModel

    # * Success if all clamps reached target or stopped
    if screwdriver.gripper_move_failed:
        logger_sync.warning("ROS_SCREWDRIVER_GRIPPER_COMMAND Failed: %s, Gripper Position = %s" % (screwdriver_id, screwdriver.currentGripperPosition))
        commander.change_command_status_and_send_to_ros(command, ROS_COMMAND.FAILED)
        return False

    # * Failure if clamp status is lost
    if current_milli_time() - screwdriver._state_timestamp > active_clamp_status_timeout_ms:
        logger_sync.warning("ROS_SCREWDRIVER_GRIPPER_COMMAND Failed: %s lost contact (timeout = %s ms)" % (screwdriver_id, active_clamp_status_timeout_ms))
        commander.change_command_status_and_send_to_ros(command, ROS_COMMAND.FAILED)
        return False

    # * Success if all clamps reached target or stopped
    if command.extend == True and screwdriver.gripper_extend_success:
        logger_sync.info("ROS_SCREWDRIVER_GRIPPER_COMMAND Completed: %s, Gripper Position = %s" % (screwdriver_id, screwdriver.currentGripperPosition))
        commander.change_command_status_and_send_to_ros(command, ROS_COMMAND.SUCCEED)
        return False

    # * Success if all clamps reached target or stopped
    if command.extend == False and screwdriver.gripper_retract_success:
        logger_sync.info("ROS_SCREWDRIVER_GRIPPER_COMMAND Completed: %s, Gripper Position = %s" % (screwdriver_id, screwdriver.currentGripperPosition))
        commander.change_command_status_and_send_to_ros(command, ROS_COMMAND.SUCCEED)
        return False

    # * Failure if clamp is not moving (checking after the success check)
    if not screwdriver.gripper_is_moving:
        logger_sync.warning("ROS_SCREWDRIVER_GRIPPER_COMMAND Failed: %s, Gripper Position = %s" % (screwdriver_id, screwdriver.currentGripperPosition))
        commander.change_command_status_and_send_to_ros(command, ROS_COMMAND.FAILED)
        return False

    # * Failure if clamp is not homed
    if not screwdriver.ishomed:
        logger_sync.warning("ROS_SCREWDRIVER_GRIPPER_COMMAND Failed: %s, Not Homed" % (screwdriver_id))
        commander.change_command_status_and_send_to_ros(command, ROS_COMMAND.FAILED)
        return False

    # * Return True if the command is still running
    return True


def initialize_logging(filename: str):
    # Logging Setup
    logger = logging.getLogger("app")
    logger.setLevel(logging.DEBUG)
    # create formatter and add it to the handlers
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    # create file handler which logs even debug messages
    log_file_handler = logging.FileHandler(filename)
    log_file_handler.setLevel(logging.DEBUG)
    log_file_handler.setFormatter(formatter)
    # create console handler with a higher log level
    log_console_handler = logging.StreamHandler()
    log_console_handler.setLevel(logging.INFO)
    log_console_handler.setFormatter(formatter)
    # add the handlers to logger
    logger.addHandler(log_file_handler)
    logger.addHandler(log_console_handler)
    logger.info("App Started")


def put_ros_command_to_queue(command: ROS_COMMAND, message: RosMessage, q=None):
    # ROS command comes from a separate thread.
    # To maintain single threaded access to the Radio / Clamp,
    # we place it in background command queue

    q.put(command)


if __name__ == "__main__":

    # Initialize Logger
    initialize_logging("TokyoCommander." + datetime.date.today().strftime("%Y-%m-%d") + ".debug.log")

    # Root TK Object
    root = tk.Tk()
    root.title("Tokyo Clamps Commander")
    root.geometry("1500x500")
    # Command queue
    q = queue.Queue()
    # Create Model
    commander = RosSerialCommander()
    # Get GUI Reference
    guiref = create_commander_gui(root, q, commander.clamps.values())

    # Start the background thread that processes UI commands
    t1 = Thread(target=background_thread, args=(guiref, commander, q))
    t1.daemon = True
    t1.start()

    # Override default ip
    guiref['ros']['ros_ip_entry'].set('192.168.0.117')
    # hostip = '192.168.43.141'
    # try:
    #     commander.ros_client = RosCommandListener(hostip, partial(put_ros_command_to_queue, q = q))
    #     commander.ros_client.run() # This starts a separate thread
    # except:
    #     logger_ctr.info(Initia)

    # Start the TK GUI Thread
    tk.mainloop()
