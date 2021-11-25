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

from clamp_controller.ClampModel import ClampModel
from clamp_controller.ScrewdriverModel import ScrewdriverModel, g_status_dict
from clamp_controller.CommanderGUI import BackgroundCommand, create_commander_gui
from clamp_controller.RosClampCommandListener import RosClampCommandListener
from clamp_controller.SerialCommander import SerialCommander


def current_milli_time(): return int(round(time.time() * 1000))

# This UI implemented a Model-View-Controller pattern.

# Model is a single SerialCommander() object
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


def background_thread(guiref, commander: SerialCommander, q):
    # This is a sudo time-split multi-task with priority execution.
    # Tasks higher up in the list have higher priority.
    # Task must return True if it was executed and return False if it was not executed.
    logging.getLogger("app.bg").info("Background Thread Started")
    while True:
        if handle_background_commands(guiref, commander, q):
            continue
        if update_status(guiref, commander):
            # Check sync move if there is a new status update.
            check_sync_move(guiref, commander)
            # Send ros statud update
            send_status_update_to_ros(commander)
            continue
    logging.getLogger("app.bg").info("Background Thread Stopped")


def get_checkbox_selected_clamps(guiref, commander: SerialCommander):
    # Determine if the clamps are selected
    clamps_selected = []
    for clamp in commander.clamps.values():
        if (guiref['status'][clamp.receiver_address]['checkbox'].get()):
            clamps_selected.append(clamp)
    return clamps_selected


def handle_background_commands(guiref, commander: SerialCommander, q):

    try:
        msg = None
        msg = q.get(timeout=0.1)
        if hasattr(msg, 'type'):
            # Handelling SERIAL_CONNECT
            if msg.type == BackgroundCommand.SERIAL_CONNECT:
                logger_ctr.info("Command Received to Connect to %s" % msg.port)
                commander.connect(msg.port)

            # Handelling UI_ROS_CONNECT
            if msg.type == BackgroundCommand.UI_ROS_CONNECT:
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
                commander.ros_client = RosClampCommandListener(msg.ip, partial(ros_command_callback, q=q))
                try:
                    commander.ros_client.run(timeout=2)  # This starts a separate thread
                    guiref['ros']['ros_status'].set("Connected to ROS")
                    logger_ctr.info("Ros Connected")
                except:
                    guiref['ros']['ros_status'].set("Not Connected")
                    pass

            # Handelling CMD_GOTO
            if msg.type == BackgroundCommand.CMD_GOTO:
                if not commander.is_connected:
                    logger_ctr.warning("Connect to Serial Radio first")
                    return True

                # Check for selected clamps
                clamps_to_communicate = get_checkbox_selected_clamps(guiref, commander)
                if len(clamps_to_communicate) == 0:
                    logger_ctr.warning("No clamp is selected for the movement command.")
                    return True

                # Instruct commander to send command
                position = msg.position
                # Beware this is not a syncronous command, because the clamp velocity might be different.
                # However, we still implement a stop_clamps() in case of messaging failure
                successes = []
                processed_clamps = []
                for clamp in clamps_to_communicate:
                    success = commander.send_clamp_to_jaw_position(clamp, position)
                    successes.append(success)
                    processed_clamps.append(clamp)
                    if not success:
                        commander.stop_clamps(processed_clamps)
                logger_ctr.info("Movement command executed. position = %smm, clamps = %s, result = %s" % (position, clamps_to_communicate, successes))

            # Handelling CMD_STOP
            if msg.type == BackgroundCommand.CMD_STOP:
                if not commander.is_connected:
                    logger_ctr.warning("Connect to Serial Radio first")
                    return True
                # Instruct commander to send command
                clamps_to_communicate = get_checkbox_selected_clamps(guiref, commander)
                if len(clamps_to_communicate) == 0:
                    return True
                result = commander.stop_clamps(clamps_to_communicate)
                logger_ctr.info("Sending stop command to %s, result = %s" % (clamps_to_communicate, result))

            # Handelling CMD_HOME
            if msg.type == BackgroundCommand.CMD_HOME:
                if not commander.is_connected:
                    logger_ctr.warning("Connect to Serial Radio first")
                    return True
                # Instruct commander to send command
                clamps_to_communicate = get_checkbox_selected_clamps(guiref, commander)
                results = commander.home_clamps(clamps_to_communicate)
                logger_ctr.info("Sending home command to %s, results = %s" % (clamps_to_communicate, results))

            # Handelling CMD_VELO
            if msg.type == BackgroundCommand.CMD_VELO:
                if not commander.is_connected:
                    logger_ctr.warning("Connect to Serial Radio first")
                    return True
                # Instruct commander to send command
                velocity = msg.velocity
                clamps_to_communicate = get_checkbox_selected_clamps(guiref, commander)
                results = commander.set_clamps_velocity(clamps_to_communicate, velocity)
                logger_ctr.info("Sending Velocity command (%s) to %s, results = %s" % (velocity, clamps_to_communicate, results))

            # Handelling CMD_POWER
            if msg.type == BackgroundCommand.CMD_POWER:
                if not commander.is_connected:
                    logger_ctr.warning("Connect to Serial Radio first")
                    return True
                # Instruct commander to send command
                power = msg.power
                clamps_to_communicate = get_checkbox_selected_clamps(guiref, commander)
                results = commander.set_clamps_power(clamps_to_communicate, power)
                logger_ctr.info("Sending Power command (%s) to %s, results = %s" % (power, clamps_to_communicate, results))

            # Handelling ROS_VEL_GOTO_COMMAND
            if msg.type == BackgroundCommand.ROS_VEL_GOTO_COMMAND:
                if not commander.is_connected:
                    logger_ctr.warning("Connect to Serial Radio first")
                    return True

                # Retrieve the list of tuples
                instructions = msg.clmap_pos_velo
                if len(instructions) == 0:
                    logger_ctr.warning("ROS_VEL_GOTO_COMMAND has no instructions")
                    return True

                # Keep track if the command is completed successful.
                # This will be set back to true by the sync watcher
                commander.last_command_success = False

                # Replace the clamp_id with the retrived ClampModel
                clamp_pos_velo_list = [(commander.clamps[clamp_id], position, velocity) for clamp_id, position, velocity in instructions]

                # Instruct commander to send command
                success = commander.sync_clamps_move(clamp_pos_velo_list)

                if success:
                    # Log
                    logger_ctr.info("ROS_VEL_GOTO_COMMAND Command Executed: Instructions = %s, results = Success" % (instructions))
                    # Send ACK back to ROS (No reply on Failed Send)
                    commander.ros_client.reply_ack_result(msg.sequence_id, success)
                else:
                    # Log
                    clamps = [commander.clamps[clamp_id] for clamp_id, position, velocity in instructions]
                    positions = [position for clamp_id, position, velocity in instructions]
                    logger_ctr.warning("ROS Command Fail: send_clamp_to_jaw_position(%s,%s) Fail" % (clamps, positions))

            # Handelling ROS_STOP_COMMAND
            if msg.type == BackgroundCommand.ROS_STOP_COMMAND:
                if not commander.is_connected:
                    logger_ctr.warning("Connect to Serial Radio first")
                    return True
                # Get the clamp objects from commander
                clamps_to_communicate = [commander.clamps[clamp_id] for clamp_id in msg.clamps_id]
                # Instruct commander to send command
                result = commander.stop_clamps(clamps_to_communicate)
                logger_ctr.info("ROS_STOP_COMMAND Executed: Stop command to %s, result = %s" % (clamps_to_communicate, result))

            # Return True if q.get() didn't return empty
            return True
    except queue.Empty:
        return False


def update_status(guiref, commander: SerialCommander):
    """Regularly calling the commander to poll clamps for a status.
    Updates the GUI labels upon a successful poll. ALso updates the last communicated time if unsuccessful.
    """
    global last_status_update_time
    if ((commander.serial_port is None) or (not commander.serial_port.isOpen())):
        return False
    if (last_status_update_time + commander.status_update_interval_ms <= current_milli_time()):
        last_status_update_time = current_milli_time()

        # * Request a status update from clamp commander
        # * If the sync movement is active, only the active clamps are polled.
        if commander.sync_move_inaction:
            updated_clamps = commander.update_active_clamps_status(1)
        else:
            updated_clamps = commander.update_all_clamps_status(1)

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
                guiref['status'][clamp.receiver_address]['last_vel'].set("%3.1fmm/s" % clamp._last_set_velocity)

            # * Update Screwdriver specific values
            if isinstance(clamp, ScrewdriverModel):
                guiref['status'][clamp.receiver_address]['grip'].set("%3.1f / %3.1f mm" % clamp.currentGripperPosition)

                if clamp._raw_gripper_status is not None:
                    guiref['status'][clamp.receiver_address]['g_status'].set(g_status_dict[clamp._raw_gripper_status])
                    if clamp._raw_gripper_status in [1, 2]:
                        guiref['status'][clamp.receiver_address]['g_status'].config(fg="green")
                    elif clamp._raw_gripper_status in [5, 6]:
                        guiref['status'][clamp.receiver_address]['g_status'].config(fg="red")
                    else:
                        guiref['status'][clamp.receiver_address]['g_status'].config(fg="black")

        return True
    else:
        return False


def check_sync_move(guiref, commander: SerialCommander, target_reach_threshold=0.5):
    """ Perhaps a checks if the commander.sync_move_inaction flag is True.
    This function checks all the clamps in commander.sync_move_clamp_pos_velo_list
    The following check is performed:
    - If any clamp in the list have not reached its target (within threshold) and have stopped moveing.
    - Maybe other checkes will be added in the future.
    This function will request the commander to stop all clamps involved.
    """
    if commander.sync_move_inaction:
        for clamp, target_jaw_position, velo in commander.sync_move_clamp_pos_velo_list:
            # Check every clamp that should be moving
            target_reached = abs(target_jaw_position - clamp.currentJawPosition) < target_reach_threshold
            if clamp.isMotorRunning == False and target_reached == False:
                logger_sync.warning("Sync Move Check Failed: %s stopped at %0.1fmm before reaching target %0.1fmm. Initializing stop all clamps." % (clamp, clamp.currentJawPosition, target_jaw_position))
                # Stop all other clamps involved in the sync move
                successes = [False]
                for _ in range(5):
                    if not all(successes):
                        successes = commander.stop_clamps([c for c, _, _ in commander.sync_move_clamp_pos_velo_list if c is not clamp])
                commander.sync_move_inaction = False
                commander.last_command_success = False
                return False
        # Cancels the flag if all clamps reached target or stopped.
        if not any([c.isMotorRunning for c, _, _ in commander.sync_move_clamp_pos_velo_list]):
            commander.sync_move_inaction = False
            logger_sync.info("Sync Move Check Completed. All clamps stopped. Flag is reset. Details:")
            for clamp, target_jaw_position, _ in commander.sync_move_clamp_pos_velo_list:
                logger_sync.info("Sync Move - %s current_pos = %0.1fmm (target = %0.1fmm)" % (clamp, clamp.currentJawPosition, target_jaw_position))
            commander.last_command_success = True
        return True
    else:
        return False


def send_status_update_to_ros(commander: SerialCommander):
    if commander.ros_client is not None:

        status = {}
        for clamp_id, clamp in commander.clamps.items():
            status[clamp_id] = clamp.state_to_data
        data = {}
        data['status'] = status
        data['last_command_success'] = commander.last_command_success
        data['sync_move_inaction'] = commander.sync_move_inaction
        commander.ros_client.send_status(data)


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


def ros_command_callback(message, q=None):
    # ROS command comes from a separate thread.
    # To maintain single threaded access to the Radio / Clamp,
    # we convert the ROS Command to a BackgroundCommand and place it in background command queue

    message_type = message['instruction_type']

    logger_ros.info("Ros Message Received: %s" % message)
    if message_type == "ROS_VEL_GOTO_COMMAND":
        sequence_id = message['sequence_id']
        instructions = message['instruction_body']
        q.put(SimpleNamespace(type=BackgroundCommand.ROS_VEL_GOTO_COMMAND, clmap_pos_velo=instructions, sequence_id=sequence_id))

    if message_type == "ROS_STOP_COMMAND":
        sequence_id = message['sequence_id']
        clamps_id = message['instruction_body']
        q.put(SimpleNamespace(type=BackgroundCommand.ROS_STOP_COMMAND, clamps_id=clamps_id, sequence_id=sequence_id))


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
    commander = SerialCommander()
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
    #     commander.ros_client = RosCommandListener(hostip, partial(ros_command_callback, q = q))
    #     commander.ros_client.run() # This starts a separate thread
    # except:
    #     logger_ctr.info(Initia)

    # Start the TK GUI Thread
    tk.mainloop()
