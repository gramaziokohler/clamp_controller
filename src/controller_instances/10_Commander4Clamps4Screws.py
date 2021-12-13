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
from clamp_controller.ScrewdriverModel import ScrewdriverModel
from clamp_controller.CommanderGUI import BackgroundCommand, create_commander_gui
from clamp_controller.SerialCommander import SerialCommander
from clamp_controller.RosClampCommandListener import RosClampCommandListener

from UI_ROS_Controller import initialize_logging, background_thread

def current_milli_time(): return int(round(time.time() * 1000))

# This UI implemented a Model-View-Controller pattern.

# Model is an object derived from `SerialCommander`. See `SerialCommanderEightTools`
# View is the TKInter UI that is created by CommanderGUI.create_commander_gui()
# Controller for the UI is the tk.mainloop().
# Controller for background task is the background_thread() that runs on a separate thread.
# - The background thread implemented a time-share based multi-task execution.
# - It is separated to handle_ui_commands() and update_status()


# Initialize multi-tasking variables.
last_status_update_time = 0

# * Model Object

class SerialCommanderEightTools(SerialCommander):

    def __init__(self):
        SerialCommander.__init__(self)

        # * Clamp Calculation
        # 918 step/mm is derived from
        # - 17 steps per rev encoder x 4 phase
        # - 1:54 gearbox
        # - 4mm lead screw

        # Soft Limit Min Max is calibrated to the CL3 clamp that was constructed.

        # Batt Min Max Value 860 to 1004 is calibrated according to the LiPo Charger's percentage reference
        # It is safe to use the battery to 0% as indicated here.
        self.add_clamp(ClampModel('c1', 'CL3', '1', 918, 0, 94.0, 225.0, 860.0, 1004.0,))
        self.add_clamp(ClampModel('c2', 'CL3', '2', 918, 0, 94.0, 225.0, 860.0, 1004.0,))
        self.add_clamp(ClampModel('c3', 'CL3M', '3', 918, 0, 94.0, 225.0, 860.0, 1004.0,))
        self.add_clamp(ClampModel('c4', 'CL3M', '4', 918, 0, 94.0, 225.0, 860.0, 1004.0,))
        self.add_clamp(ScrewdriverModel('s1', 'SL1', '5', 5685.6, 0, -10, 300, 860.0, 1004.0, 2501.9))
        self.add_clamp(ScrewdriverModel('s2', 'SL1', '6', 5685.6, 0, -10, 300, 860.0, 1004.0, 2501.9))
        self.add_clamp(ScrewdriverModel('s3', 'SL1', '7', 5685.6, 0, -10, 300, 860.0, 1004.0, 2501.9))
        self.add_clamp(ScrewdriverModel('s4', 'SL1_G200', '8', 5685.6, 0, -10, 300, 860.0, 1004.0, 2501.9))

        self.ros_client: RosClampCommandListener = None
        self.status_update_interval_low_ms = 950


if __name__ == "__main__":

    # *Initialize Logger
    initialize_logging("EightToolCommander." + datetime.date.today().strftime("%Y-%m-%d") + ".debug.log")

    # * Commander Model
    commander = SerialCommanderEightTools() # Create Model

    # * Code to create Controller Object. It returns guiref dictionary for background thread to act on
    root = tk.Tk() # Root TK Object
    root.title("Tokyo Clamps Commander")
    root.geometry("1800x800")
    q = queue.Queue() # Command queue
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
