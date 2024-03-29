import logging
import tkinter as tk
import tkinter.font as tkFont
from enum import Enum, auto
from queue import Queue
from tkinter import ttk
from typing import List, Tuple
from types import SimpleNamespace

from clamp_controller.ClampModel import ClampModel
from clamp_controller.ScrewdriverModel import ScrewdriverModel
from clamp_controller.RosCommand import *
from serial.tools import list_ports

logger_ui = logging.getLogger("app.UI")


class ClampControllerBackgroundCommand(Enum):
    # Misc UI Control
    UI_SERIAL_CONNECT = auto()
    UI_ROS_CONNECT = auto()
    LOGGING = auto()
    # Manually issued movement command
    CMD_POWER = auto()
    CMD_HOME = auto()
    CMD_STOP = auto()
    CMD_STOP_ONE = auto()
    CMD_CLAMP_GOTO = auto()
    CMD_CLAMP_VELO = auto()
    CMD_CLAMP_OVERRIDE_CURRENT_POS = auto()
    CMD_SCREWDRIVER_GOTO = auto()
    CMD_SCREWDRIVER_VELO = auto()
    CMD_SCREWDRIVER_OVERRIDE_CURRENT_POS = auto()
    CMD_SCREWDRIVER_GRIPPER = auto()


def create_commander_gui(root, q: Queue, clamps):
    tk.font_key = tkFont.Font(family="Lucida Grande", size=10)
    tk.font_value = tkFont.Font(family="Lucida Console", size=20)
    tk.font_title = tkFont.Font(family="Lucida Grande", size=12)
    tk.font_description = tkFont.Font(family="Lucida Grande", size=8)

    ui_handles = {}
    ui_handles['connect'] = create_ui_connect(root, q)
    ui_handles['status'] = create_ui_status(root, q, clamps)
    ui_handles['control'] = create_ui_control(root, q)
    #ui_handles['logging'] = create_ui_logging(root, q)
    ui_handles['ros'] = create_ui_ros(root, q)
    return ui_handles


def create_ui_connect(root, q: Queue):
    ui_handles = {}

    # * Title frame
    frame = ttk.Frame(root, borderwidth=0)
    frame.pack(fill=tk.BOTH, expand=0, side=tk.TOP, padx=6, pady=3)
    tk.Label(frame, text="Connection ", font=tk.font_title).pack(side=tk.LEFT, fill=tk.Y)
    tk.Label(frame, text=" (Serial Port for Radio Dongle)", font=tk.font_description).pack(side=tk.LEFT, fill=tk.Y)

    # Title and frame
    frame = ttk.Frame(root, borderwidth=2, relief='solid')
    frame.pack(fill=tk.BOTH, expand=0, side=tk.TOP, padx=6, pady=3)

    # Label
    label_1 = tk.Label(frame, text="Serial Port for USB Radio")
    label_1.pack(side=tk.LEFT)

    # Combo box
    def list_serial_ports():
        return list_ports.comports()

    def cb_update_serial_ports():
        serial_cb["values"] = list_serial_ports()
    serial_cb = ttk.Combobox(frame, width=40, values=[], postcommand=cb_update_serial_ports)
    serial_cb.pack(side=tk.LEFT, padx=10)
    ui_handles['serial_cb'] = serial_cb

    # Button
    def on_connect_button_click(event=None):
        cb_value = serial_cb.get()
        ports = list_serial_ports()
        # Loop though ports to find the selected port object
        logger_ui.info("Button Pressed: Connect Serial")
        for port in ports:
            if (cb_value == port.__str__()):
                logger_ui.info("Selected Port: %s" % port[0])
                q.put(SimpleNamespace(type=ClampControllerBackgroundCommand.UI_SERIAL_CONNECT, port=port[0]))
                break
    button = tk.Button(frame, text="Connect / Reconnect", command=on_connect_button_click)
    button.pack(side=tk.LEFT)

    return ui_handles


def create_ui_status(root, q: Queue, clamps):
    ui_handles = {}

    # * Title frame
    frame = ttk.Frame(root, borderwidth=0)
    frame.pack(fill=tk.BOTH, expand=0, side=tk.TOP, padx=6, pady=3)
    tk.Label(frame, text="Status ", font=tk.font_title).pack(side=tk.LEFT, fill=tk.Y)
    tk.Label(frame, text=" (Checked devices are regularly polled)", font=tk.font_description).pack(side=tk.LEFT, fill=tk.Y)

    # * Create one row of status per clamp
    frame = ttk.Frame(root, borderwidth=2, relief='solid')
    frame.pack(fill=tk.BOTH, expand=0, side=tk.TOP, padx=6, pady=3)

    for clamp in clamps:
        ui_handles[clamp.receiver_address] = create_one_ui_status(frame, q, clamp)

    return ui_handles


def create_one_ui_status(root, q: Queue, clamp: ClampModel):
    ui_handles = {}

    # Within that one row, We pack left with fixed width
    frame = ttk.Frame(root, borderwidth=2, relief='solid')
    frame.pack(fill=tk.BOTH, expand=0, side=tk.TOP, padx=6, pady=3)

    tk.font_key = tkFont.Font(family="Lucida Grande", size=8)
    tk.font_value = tkFont.Font(family="Lucida Grande", size=16)

    tk.Label(frame, text="com?", font=tk.font_key, anchor=tk.SE).pack(side=tk.LEFT, fill=tk.Y)
    ui_handles['checkbox'] = tk.BooleanVar(value=True)
    tk.Checkbutton(frame, variable=ui_handles['checkbox']).pack(side=tk.LEFT, padx=10)

    def create_label_pair(label_text, textvariable_name, label_name=None):
        tk.Label(frame, text=label_text, font=tk.font_key, anchor=tk.SE).pack(side=tk.LEFT, fill=tk.Y)
        ui_handles[textvariable_name] = tk.StringVar()
        ui_handles[textvariable_name].set("<?>")
        if label_name is None:
            label_name = textvariable_name + "_label"
        ui_handles[label_name] = tk.Label(frame, textvariable=ui_handles[textvariable_name], font=tk.font_value)
        ui_handles[label_name].pack(side=tk.LEFT, fill=tk.Y, padx=5)

    # Create the fields
    create_label_pair("addr", "addr")
    ui_handles["addr"].set(clamp.receiver_address)
    create_label_pair("tool_id", "tool_id")
    ui_handles["tool_id"].set(clamp.process_tool_id)
    create_label_pair("type", "type")
    ui_handles["type"].set(clamp.typeName)
    create_label_pair("home", "home")
    create_label_pair("motion", "motion")
    create_label_pair("pos", "position")
    create_label_pair("err", "error")
    create_label_pair("power", "power")
    create_label_pair("batt", "battery")
    create_label_pair("last_com", "last_com")
    create_label_pair("last_pos", "last_pos")
    create_label_pair("last_vel", "last_vel")

    def on_home_button_click(receiver_address: str):
        logger_ui.info("Device HOME Button Pressed: %s" % receiver_address)
        q.put(SimpleNamespace(type=ClampControllerBackgroundCommand.CMD_HOME, receiver_address=receiver_address))
    tk.Button(frame, text="Home", command=lambda: on_home_button_click(clamp.receiver_address)).pack(side=tk.LEFT, padx=5)

    def on_stop_button_click(receiver_address: str):
        logger_ui.info("Device STOP Button Pressed: %s" % receiver_address)
        q.put(SimpleNamespace(type=ClampControllerBackgroundCommand.CMD_STOP_ONE, receiver_address=receiver_address))
    tk.Button(frame, text="Stop", command=lambda: on_stop_button_click(clamp.receiver_address)).pack(side=tk.LEFT, padx=5)


    if isinstance(clamp, ClampModel):
        def on_camera_reset_button_click(receiver_address: str):
            logger_ui.info("Device Camera Reset Button Pressed (Not Implemented): %s" % receiver_address)
            # q.put(SimpleNamespace(type=ClampControllerBackgroundCommand.CAM_RESET, receiver_address=receiver_address))
        tk.Button(frame, text="Camera Reset", command=lambda: on_camera_reset_button_click(clamp.receiver_address)).pack(side=tk.LEFT, padx=5)

    if isinstance(clamp, ScrewdriverModel):
        create_label_pair("grip", "grip")
        create_label_pair("g_status", "g_status")

        def on_gripper_button_click(receiver_address: str, extend: bool):
            if extend:
                logger_ui.info("Screwdriver Gripper Extend Button Pressed: %s" % receiver_address)
            else:
                logger_ui.info("Screwdriver Gripper Retract Button Pressed: %s" % receiver_address)
            q.put(SimpleNamespace(type=ClampControllerBackgroundCommand.CMD_SCREWDRIVER_GRIPPER, extend=extend, receiver_address=receiver_address))

        tk.Button(frame, text="Extend", command=lambda: on_gripper_button_click(extend=True, receiver_address=clamp.receiver_address)).pack(side=tk.LEFT, padx=5)
        tk.Button(frame, text="Retract", command=lambda: on_gripper_button_click(extend=False, receiver_address=clamp.receiver_address)).pack(side=tk.LEFT, padx=5)

    return ui_handles


def create_ui_control(root, q: Queue):

    ui_handles = {}

    # * Title frame
    frame = ttk.Frame(root, borderwidth=0)
    frame.pack(fill=tk.BOTH, expand=0, side=tk.TOP, padx=6, pady=3)
    tk.Label(frame, text="Device Control ", font=tk.font_title).pack(side=tk.LEFT, fill=tk.Y)
    tk.Label(frame, text=" (Manual Commands are only sent to Checked)", font=tk.font_description).pack(side=tk.LEFT, fill=tk.Y)

    frame_outside = ttk.Frame(root, borderwidth=2, relief='solid')
    frame_outside.pack(fill=tk.BOTH, expand=0, side=tk.TOP, padx=6, pady=3)

    # * Row for Clamp Control
    # * ---------------------
    frame = ttk.Frame(frame_outside, borderwidth=2, relief='solid')
    frame.pack(fill=tk.BOTH, expand=0, side=tk.TOP, padx=6, pady=3)

    # Buttons - Clamp Go To Position
    def on_clamp_goto_button_click(position):
        logger_ui.info("Button Pressed: Clamp Go to Position %s" % position)
        q.put(SimpleNamespace(type=ClampControllerBackgroundCommand.CMD_CLAMP_GOTO, position=position))

    tk.Label(frame, text="Clamp: Goto Position: ", font=tk.font_key, anchor=tk.SE).pack(side=tk.LEFT, fill=tk.Y)
    tk.Button(frame, text="101mm", command=lambda: on_clamp_goto_button_click(101)).pack(side=tk.LEFT)
    tk.Button(frame, text="102mm", command=lambda: on_clamp_goto_button_click(102)).pack(side=tk.LEFT)
    tk.Button(frame, text="110mm", command=lambda: on_clamp_goto_button_click(110)).pack(side=tk.LEFT)
    tk.Button(frame, text="140mm", command=lambda: on_clamp_goto_button_click(140)).pack(side=tk.LEFT)
    tk.Button(frame, text="155mm", command=lambda: on_clamp_goto_button_click(155)).pack(side=tk.LEFT)
    tk.Button(frame, text="170mm", command=lambda: on_clamp_goto_button_click(170)).pack(side=tk.LEFT)
    tk.Button(frame, text="200mm", command=lambda: on_clamp_goto_button_click(200)).pack(side=tk.LEFT)
    tk.Button(frame, text="220mm", command=lambda: on_clamp_goto_button_click(220)).pack(side=tk.LEFT)

    ui_handles['custom_pos_clamp'] = tk.StringVar(value="100.5")
    tk.Entry(frame, textvariable=ui_handles['custom_pos_clamp'], width=10,  justify=tk.CENTER).pack(side=tk.LEFT)
    tk.Button(frame, text="mm (Custom Pos)", command=lambda: on_clamp_goto_button_click(float(ui_handles['custom_pos_clamp'].get()))).pack(side=tk.LEFT)

    # Buttons - Clamp Set Velocity
    def on_clamp_velo_button_click(velocity):
        logger_ui.info("Button Pressed: Set Velocity %s" % velocity)
        q.put(SimpleNamespace(type=ClampControllerBackgroundCommand.CMD_CLAMP_VELO, velocity=velocity))

    tk.Label(frame, text="Set Velocity: ", font=tk.font_key, anchor=tk.SE).pack(side=tk.LEFT, fill=tk.Y, padx=10)
    tk.Button(frame, text="1mm/s", command=lambda: on_clamp_velo_button_click(1)).pack(side=tk.LEFT)
    tk.Button(frame, text="2mm/s", command=lambda: on_clamp_velo_button_click(2)).pack(side=tk.LEFT)
    tk.Button(frame, text="3mm/s", command=lambda: on_clamp_velo_button_click(3)).pack(side=tk.LEFT)
    tk.Button(frame, text="4mm/s", command=lambda: on_clamp_velo_button_click(4)).pack(side=tk.LEFT)
    tk.Button(frame, text="5mm/s", command=lambda: on_clamp_velo_button_click(5)).pack(side=tk.LEFT)

    ui_handles['custom_vel_clamp'] = tk.StringVar(value="2.5")
    tk.Entry(frame, textvariable=ui_handles['custom_vel_clamp'], width=10, justify=tk.CENTER).pack(side=tk.LEFT)
    tk.Button(frame, text="mm/s (Custom Vel)", command=lambda: on_clamp_velo_button_click(float(ui_handles['custom_vel_clamp'].get()))).pack(side=tk.LEFT)

    # Buttons - Clamp Override current Position
    def on_clamp_set_pos_button_click(position):
        logger_ui.info("Button Pressed: Override current Position %s" % position)
        q.put(SimpleNamespace(type=ClampControllerBackgroundCommand.CMD_CLAMP_OVERRIDE_CURRENT_POS, position=position))

    tk.Label(frame, text="Override Current Position: ", font=tk.font_key, anchor=tk.SE).pack(side=tk.LEFT, fill=tk.Y, padx=10)
    ui_handles['override_val_clamp'] = tk.StringVar(value="100")
    tk.Entry(frame, textvariable=ui_handles['override_val_clamp'], width=10, justify=tk.CENTER).pack(side=tk.LEFT)
    tk.Button(frame, text="mm (override)", command=lambda: on_clamp_set_pos_button_click(float(ui_handles['override_val_clamp'].get()))).pack(side=tk.LEFT)



    # * Row for Screwdriver Control
    # * ---------------------
    frame = ttk.Frame(frame_outside, borderwidth=2, relief='solid')
    frame.pack(fill=tk.BOTH, expand=0, side=tk.TOP, padx=6, pady=3)

    # Buttons - Screwdriver Go To Postion
    def on_screwdriver_goto_button_click(position):
        logger_ui.info("Button Pressed: Screwdriver Go to Position %s" % position)
        q.put(SimpleNamespace(type=ClampControllerBackgroundCommand.CMD_SCREWDRIVER_GOTO, position=position))

    tk.Label(frame, text="Screwdriver: Goto Position: ", font=tk.font_key, anchor=tk.SE).pack(side=tk.LEFT, fill=tk.Y)
    tk.Button(frame, text="0mm", command=lambda: on_screwdriver_goto_button_click(0)).pack(side=tk.LEFT)
    tk.Button(frame, text="5mm", command=lambda: on_screwdriver_goto_button_click(5)).pack(side=tk.LEFT)
    tk.Button(frame, text="50mm", command=lambda: on_screwdriver_goto_button_click(50)).pack(side=tk.LEFT)
    tk.Button(frame, text="100mm", command=lambda: on_screwdriver_goto_button_click(100)).pack(side=tk.LEFT)
    tk.Button(frame, text="150mm", command=lambda: on_screwdriver_goto_button_click(150)).pack(side=tk.LEFT)
    tk.Button(frame, text="200mm", command=lambda: on_screwdriver_goto_button_click(200)).pack(side=tk.LEFT)
    tk.Button(frame, text="250mm", command=lambda: on_screwdriver_goto_button_click(250)).pack(side=tk.LEFT)

    ui_handles['custom_pos_screwdriver'] = tk.StringVar(value="10.5")
    tk.Entry(frame, textvariable=ui_handles['custom_pos_screwdriver'], width=10,  justify=tk.CENTER).pack(side=tk.LEFT)
    tk.Button(frame, text="mm (Custom Pos)", command=lambda: on_screwdriver_goto_button_click(float(ui_handles['custom_pos_screwdriver'].get()))).pack(side=tk.LEFT)

    # Buttons - Screwdriver Set Velocity
    def on_screwdriver_velo_button_click(velocity):
        logger_ui.info("Button Pressed: Set Velocity %s" % velocity)
        q.put(SimpleNamespace(type=ClampControllerBackgroundCommand.CMD_SCREWDRIVER_VELO, velocity=velocity))

    tk.Label(frame, text="Set Velocity: ", font=tk.font_key, anchor=tk.SE).pack(side=tk.LEFT, fill=tk.Y, padx=10)
    tk.Button(frame, text="0.70mm/s", command=lambda: on_screwdriver_velo_button_click(0.70)).pack(side=tk.LEFT)
    tk.Button(frame, text="0.75mm/s", command=lambda: on_screwdriver_velo_button_click(0.75)).pack(side=tk.LEFT)
    tk.Button(frame, text="0.80mm/s", command=lambda: on_screwdriver_velo_button_click(0.80)).pack(side=tk.LEFT)
    tk.Button(frame, text="0.85mm/s", command=lambda: on_screwdriver_velo_button_click(0.85)).pack(side=tk.LEFT)
    tk.Button(frame, text="0.90mm/s", command=lambda: on_screwdriver_velo_button_click(0.90)).pack(side=tk.LEFT)

    ui_handles['custom_vel_screwdriver'] = tk.StringVar(value="0.80")
    tk.Entry(frame, textvariable=ui_handles['custom_vel_screwdriver'], width=10, justify=tk.CENTER).pack(side=tk.LEFT)
    tk.Button(frame, text="mm/s (Custom Vel)", command=lambda: on_screwdriver_velo_button_click(float(ui_handles['custom_vel_screwdriver'].get()))).pack(side=tk.LEFT)

    # Buttons - Screwdriver Override current Position
    def on_screwdriver_set_pos_button_click(position):
        logger_ui.info("Button Pressed: Override current Position %s" % position)
        q.put(SimpleNamespace(type=ClampControllerBackgroundCommand.CMD_SCREWDRIVER_OVERRIDE_CURRENT_POS, position=position))

    tk.Label(frame, text="Override Current Position: ", font=tk.font_key, anchor=tk.SE).pack(side=tk.LEFT, fill=tk.Y, padx=10)
    ui_handles['override_val_screwdriver'] = tk.StringVar(value="0")
    tk.Entry(frame, textvariable=ui_handles['override_val_screwdriver'], width=10, justify=tk.CENTER).pack(side=tk.LEFT)
    tk.Button(frame, text="mm (override)", command=lambda: on_screwdriver_set_pos_button_click(float(ui_handles['override_val_screwdriver'].get()))).pack(side=tk.LEFT)

    # Buttons - Screwdriver Gripper Extend Retract
    def on_gripper_button_click(extend: bool):
        if extend:
            logger_ui.info("Button Pressed: Gripper Extend")
        else:
            logger_ui.info("Button Pressed: Gripper Retract")
        q.put(SimpleNamespace(type=ClampControllerBackgroundCommand.CMD_SCREWDRIVER_GRIPPER, extend=extend))

    tk.Label(frame, text="Pin Gripper", font=tk.font_key, anchor=tk.SE).pack(side=tk.LEFT, fill=tk.Y, padx=10)
    tk.Button(frame, text="Extend / Grip", command=lambda: on_gripper_button_click(True)).pack(side=tk.LEFT)
    tk.Button(frame, text="Retract / Release", command=lambda: on_gripper_button_click(False)).pack(side=tk.LEFT)

    # * Row for Power Setting
    # * ---------------------

    def on_power_button_click(power):
        logger_ui.info("Button Pressed: Set Power %s" % power)
        q.put(SimpleNamespace(type=ClampControllerBackgroundCommand.CMD_POWER, power=power))

    frame = ttk.Frame(frame_outside, borderwidth=2, relief='solid')
    frame.pack(fill=tk.BOTH, expand=0, side=tk.TOP, padx=6, pady=3)

    tk.Label(frame, text="Set Power: ", font=tk.font_key, anchor=tk.SE).pack(side=tk.LEFT, fill=tk.Y, padx=00)
    tk.Button(frame, text="20%", command=lambda: on_power_button_click(20)).pack(side=tk.LEFT)
    tk.Button(frame, text="40%", command=lambda: on_power_button_click(40)).pack(side=tk.LEFT)
    tk.Button(frame, text="50%", command=lambda: on_power_button_click(50)).pack(side=tk.LEFT)
    tk.Button(frame, text="60%", command=lambda: on_power_button_click(60)).pack(side=tk.LEFT)
    tk.Button(frame, text="65%", command=lambda: on_power_button_click(65)).pack(side=tk.LEFT)
    tk.Button(frame, text="70%", command=lambda: on_power_button_click(70)).pack(side=tk.LEFT)
    tk.Button(frame, text="75%", command=lambda: on_power_button_click(75)).pack(side=tk.LEFT)
    tk.Button(frame, text="80%", command=lambda: on_power_button_click(80)).pack(side=tk.LEFT)
    tk.Button(frame, text="90%", command=lambda: on_power_button_click(90)).pack(side=tk.LEFT)
    tk.Button(frame, text="99%", command=lambda: on_power_button_click(99)).pack(side=tk.LEFT)

    # * Row for Power Setting
    # * ---------------------
    frame = ttk.Frame(frame_outside, borderwidth=2, relief='solid')
    frame.pack(fill=tk.BOTH, expand=0, side=tk.TOP, padx=6, pady=3)

    def on_stop_button_click():
        logger_ui.info("Button Pressed: STOP")
        q.put(SimpleNamespace(type=ClampControllerBackgroundCommand.CMD_STOP))

    tk.Label(frame, text="MASTER STOP: ", font=tk.font_title, anchor=tk.SE).pack(side=tk.LEFT, fill=tk.Y, padx=10)
    tk.Button(frame, text="! STOP ALL !", font=tk.font_title, command=on_stop_button_click).pack(side=tk.LEFT)

    def on_home_button_click():
        logger_ui.info("Button Pressed: HOME")
        q.put(SimpleNamespace(type=ClampControllerBackgroundCommand.CMD_HOME))

    tk.Label(frame, text="HOME Selected: ", font=tk.font_title, anchor=tk.SE).pack(side=tk.LEFT, fill=tk.Y, padx=10)
    tk.Button(frame, text="Home", font=tk.font_title, command=on_home_button_click).pack(side=tk.LEFT)

    return ui_handles


def create_ui_logging(root, q: Queue):

    ui_handles = {}

    font_key = tkFont.Font(family="Lucida Grande", size=10)
    font_value = tkFont.Font(family="Lucida Grande", size=20)

    # Title and frame
    title = tk.Label(root, text="Logging")
    title.pack(anchor=tk.NW, expand=0, side=tk.TOP, padx=3, pady=3)
    frame = ttk.Frame(root, borderwidth=2, relief='solid')
    frame.pack(fill=tk.BOTH, expand=0, side=tk.TOP, padx=6, pady=3)

    # Button
    def on_logging_button_click(event=None):
        logger_ui.info("Button Pressed: logging_button")
        q.put(SimpleNamespace(type=ClampControllerBackgroundCommand.LOGGING))

    ui_handles['logging_button_text'] = tk.StringVar(value="Start Logging")
    tk.Button(frame, textvariable=ui_handles['logging_button_text'], command=on_logging_button_click).pack(side=tk.LEFT)
    return ui_handles


def create_ui_ros(root, q: Queue):

    ui_handles = {}

    # * Title frame
    frame = ttk.Frame(root, borderwidth=0)
    frame.pack(fill=tk.BOTH, expand=0, side=tk.TOP, padx=6, pady=3)
    tk.Label(frame, text="ROS Connection ", font=tk.font_title).pack(side=tk.LEFT, fill=tk.Y)
    tk.Label(frame, text=" (RFL ROS:192.168.0.117, LOCAL ROS:192.168.1.2)", font=tk.font_description).pack(side=tk.LEFT, fill=tk.Y)

    frame = ttk.Frame(root, borderwidth=2, relief='solid')
    frame.pack(fill=tk.BOTH, expand=0, side=tk.TOP, padx=6, pady=3)

    # Button Handle
    def on_ros_connect_button_click(event=None):
        logger_ui.info("Button Pressed: Connect to ROS")
        ip = ros_ip_entrybox.get()
        q.put(SimpleNamespace(type=ClampControllerBackgroundCommand.UI_ROS_CONNECT, ip=ip))

    tk.Label(frame, text="ROS Core IP Address: ", font=tk.font_key, anchor=tk.SE).pack(side=tk.LEFT, fill=tk.Y, padx=10)
    ui_handles['ros_ip_entry'] = tk.StringVar(value="127.0.0.0")
    ros_ip_entrybox = tk.Entry(frame, textvariable=ui_handles['ros_ip_entry'])
    ros_ip_entrybox.pack(side=tk.LEFT)
    tk.Button(frame, text="Connect", command=on_ros_connect_button_click).pack(side=tk.LEFT)
    # Status Label
    tk.Label(frame, text="Status: ", font=tk.font_key, anchor=tk.SE).pack(side=tk.LEFT, fill=tk.Y, padx=10)
    ui_handles['ros_status'] = tk.StringVar(value="Not Connected")
    tk.Label(frame, textvariable=ui_handles['ros_status'], font=tk.font_key, anchor=tk.SE).pack(side=tk.LEFT, fill=tk.Y, padx=10)

    return ui_handles
