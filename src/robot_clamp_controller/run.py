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
from clamp_controller.RemoteClampFunctionCall import RemoteClampFunctionCall

from robot_clamp_controller.BackgroundCommand import *
from robot_clamp_controller.GUI import *
from robot_clamp_controller.ProcessModel import RobotClampExecutionModel, RunStatus
from robot_clamp_controller.Execute import execute_movement
import argparse


def current_milli_time(): return int(round(time.time() * 1000))

# This UI implemented a Model-View-Controller pattern.
# Described in https://maldus512.medium.com/how-to-setup-correctly-an-application-with-python-and-tkinter-107c6bc5a45

# Model is a single RobotClampExecutionModel() object
# View is the TKInter UI that is created by robot_clamp_controller.GUI.create_execution_gui()
# Controller for the UI is the tk.mainloop().
# Controller for background task is the background_thread() that runs on a separate thread.
# - The background thread implemented a time-share based multi-task execution.
# - It is separated to handle_ui_commands() and update_status()


# Initialize logger for the controller

logger_bg = logging.getLogger("app.bg")
logger_run = logging.getLogger("app.run")
logger_ctr = logging.getLogger("app.ctr")
logger_ros = logging.getLogger("app.ros")
logger_sync = logging.getLogger("app.sync")
# Initialize multi-tasking variables.
last_status_update_time = 0


def background_thread(guiref, model: RobotClampExecutionModel, q):
    """ Background thread to handel UI commands and other tasks.
    Tasks higher up in the list have higher priority.

    Task must return True if it was executed and return False if it was not executed."""
    logger_bg.info("Background Thread Started")
    while True:
        if handle_background_commands(guiref, model, q):
            continue
    logger_bg.info("Background Thread Stopped")


def handle_background_commands(guiref, model: RobotClampExecutionModel, q):
    try:
        msg = None
        msg = q.get(timeout=0.1)
        if hasattr(msg, 'type'):

            # Handelling MODEL_LOAD_PROCESS upon closure of dialog
            if msg.type == BackgroundCommand.MODEL_LOAD_PROCESS:
                logger_bg.info(
                    "Relaying BackgroundCommand: MODEL_LOAD_PROCESS")
                model.load_process(msg.json_path)
                if model is not None:
                    guiref['process']['process_status'].set(
                        model.process_description)
                init_actions_tree_view(guiref, model)
            
            # Handle UI_UPDATE_STATUS
            if msg.type == BackgroundCommand.UI_UPDATE_STATUS:
                logger_bg.info(
                    "ui_update_run_status")
                ui_update_run_status(guiref, model)

            # Handelling UI_RUN
            if msg.type == BackgroundCommand.UI_RUN:
                logger_bg.info(
                    "Relaying BackgroundCommand: UI_RUN, Starting new RUN thread.")
                if model.process is None:
                    logger_bg.info("Load Process first")
                elif not model.run_status == RunStatus.RUNNING:
                    model.run_status = RunStatus.RUNNING
                    model.run_thread = Thread(target=program_run_thread, args=(guiref, model, q), daemon=True)
                    model.run_thread.start()
                    ui_update_run_status(guiref, model)
                # Dont do anyhting if program is already running

            # Handelling UI_STEP
            if msg.type == BackgroundCommand.UI_STEP:
                logger_bg.info(
                    "Relaying BackgroundCommand: UI_STEP")
                if model.process is None:
                    logger_bg.info("Load Process first")
                else:
                    # Change Status
                    model.run_status = RunStatus.STEPPING_FORWARD
                    # Start a new thread if the current one is not active
                    if model.run_thread is None or not model.run_thread.isAlive():
                        model.run_thread = Thread(target=program_run_thread, args=(guiref, model, q), daemon=True)
                        model.run_thread.start()
                    ui_update_run_status(guiref, model)

            # Handelling UI_STOP
            if msg.type == BackgroundCommand.UI_STOP:
                logger_bg.info(
                    "Relaying BackgroundCommand: UI_STOP. Stopping RUN THREAD")
                model.run_status = RunStatus.STOPPED
                ui_update_run_status(guiref, model)

            # Handelling UI_CONFIRM
            if msg.type == BackgroundCommand.UI_CONFIRM:
                logger_bg.info(
                    "Relaying BackgroundCommand: UI_CONFIRM.")
                model.operator_confirm = True
                ui_update_run_status(guiref, model)

            # Handelling UI_ROS_CONNECT
            if msg.type == BackgroundCommand.UI_ROS_CONNECT:
                logger_bg.info(
                    "Relaying BackgroundCommand: UI_ROS_CONNECT - msg.ip = %s" % msg.ip)

                # Connect to new ROS host
                guiref['ros']['ros_status'].set("Connecting to ROS")
                if model.connect_ros_clamps(msg.ip, q):
                    guiref['ros']['ros_status'].set("Connected to ROS")
                    logger_ctr.info("Ros Connected")
                else:
                    guiref['ros']['ros_status'].set("Not Connected")


            return True
    except queue.Empty:
        return False


def wait_for_opeartor_confirm(guiref, model:RobotClampExecutionModel, message : str = "Confirm"):
    """This is a blocking call that enables the confirm button witgh a message. 
    Returns True if Operator presses the button.
    Returns False if model.run_status changes to STOPPED indicating a stop. 
    """
    button = guiref['exe']['confirm_button']
    guiref['exe']['confirm_button_text'].set(message)
    button.config(state="normal", bg='orange')
    model.operator_confirm = False

    guiref['exe']['exe_status'].set("Paused")
    while (True):
        if model.operator_confirm:
            button.config(state="disabled", bg='grey')
            ui_update_run_status(guiref, model)
            return True
        if model.run_status == RunStatus.STOPPED:
            button.config(state="disabled", bg='grey')
            ui_update_run_status(guiref, model)
            return False
    

def program_run_thread(guiref, model: RobotClampExecutionModel, q):
    """ Thread for running programms. This thread exist when Run or Step is pressed.
    This thread will run at least one movement, and be terminated if:
    - model.run_status

    Care should be taken not to have opened resources that cannot be released
    as this thread could be terminated abrubtly.
    """
    logger_run.info("Program Run Thread Started")
    while True:

        # Run the Selected Item. If it is not a Movement, select the next movement.
        move_id = treeview_get_selected_id(guiref)
        if not move_id.startswith('m'):
            move_id = treeview_select_next_movement(guiref)

        movement = model.movements[move_id] # type: Movement

        # Pause before
        if movement.operator_stop_before != "":
            confirm = wait_for_opeartor_confirm(guiref, model, movement.operator_stop_before)
            if not confirm: 
                logger_run.info("Operator stop not confirmed before movement. Run Thread Ended.")
                q.put(SimpleNamespace(type=BackgroundCommand.UI_UPDATE_STATUS))
                break
        
        # Execution
        success = execute_movement(model, movement)

        # Pause after
        if movement.operator_stop_after != "":
            confirm = wait_for_opeartor_confirm(guiref, model, movement.operator_stop_after)
            if not confirm: 
                logger_run.info("Operator stop not confirmed after movement. Run Thread Ended.")
                q.put(SimpleNamespace(type=BackgroundCommand.UI_UPDATE_STATUS))
                break

        # Terminate if execution is not success, do not increment pointer.
        if not success:
            model.run_status == RunStatus.ERROR
            logger_run.info("Execution Error. Program Stopped. Run Thread Ended.")
            q.put(SimpleNamespace(type=BackgroundCommand.UI_UPDATE_STATUS))
            break

        # Move program pointer to next movement
        treeview_select_next_movement(guiref)

        # Stop execution if it is currently in STEPPING mode.
        if model.run_status == RunStatus.STEPPING_FORWARD:
            model.run_status = RunStatus.STOPPED

        # Status note that the RUN Thread is Stopped
        if model.run_status == RunStatus.STOPPED:
            logger_run.info("Program Stopped. Run Thread Ended.")
            q.put(SimpleNamespace(type=BackgroundCommand.UI_UPDATE_STATUS))
            break


def initialize_logging(filename: str):
    # Logging Setup
    logger = logging.getLogger("app")
    logger.setLevel(logging.DEBUG)
    # create formatter and add it to the handlers
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s')
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





if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='CLI RobotClampExecution.')
    parser.add_argument('-f', default='',  help='Load Process File on Start')
    parser.add_argument('-rosip', default='192.168.0.117',  help='Load Process File on Start')
    args = parser.parse_args()
    print(args)
    # Initialize Logger
    initialize_logging("RobotClampExecution." +
                       datetime.date.today().strftime("%Y-%m-%d") + ".debug.log")

    # Root TK Object
    root = tk.Tk()
    root.title("Robot and Clamps Assembly Process Execution")
    root.geometry("1500x600")

    # Command queue
    q = queue.Queue()
    # Create Model
    model = RobotClampExecutionModel()
    # Get GUI Reference
    guiref = create_execution_gui(root, q)

    # Start the background thread that processes UI commands
    t1 = Thread(target=background_thread, args=(guiref, model, q))
    t1.daemon = True
    t1.start()

    # Override default ip
    guiref['ros']['ros_ip_entry'].set(args.rosip)  # VM Address
    # guiref['ros']['ros_ip_entry'].set('192.168.0.117') # RFL Address

    if not args.f == "":
        logger_ctr.info(
            "Command Line contain -f. Load Json File at %s." % args.f)
        q.put(SimpleNamespace(
            type=BackgroundCommand.MODEL_LOAD_PROCESS, json_path=args.f))

    # Start the TK GUI Thread
    tk.mainloop()


# Development command line start with sample file opened:
# python C:\Users\leungp\Documents\GitHub\clamp_controller\src\robot_clamp_controller\run.py -f C:\Users\leungp\Documents\GitHub\itj_design_study\210128_RemodelFredPavilion\twelve_pieces_process.json -rosip 192.168.20.128