import logging
import tkinter as tk
import tkinter.font as tkFont
from queue import Queue
from tkinter import filedialog, ttk
from types import SimpleNamespace
from typing import Dict, List, Optional, Tuple

from integral_timber_joints.process.action import *
from integral_timber_joints.process.movement import *

from robot_clamp_controller.ProcessModel import (RobotClampExecutionModel,
                                                 RunStatus)
from robot_clamp_controller.BackgroundCommand import *
logger_ui = logging.getLogger("app.gui")


def create_execution_gui(root, q):
    tk.font_key = tkFont.Font(family="Lucida Grande", size=10)
    tk.font_value = tkFont.Font(family="Lucida Console", size=20)
    tk.big_button_font = tkFont.Font(family="Lucida Console", size=15)
    tk.big_status_font = tkFont.Font(
        family="Lucida Console", size=25, weight='bold')

    ui_handles = {}
    # ui_handles['connect'] = create_ui_connect(root, q)
    # ui_handles['status'] = create_ui_status(root, q, clamps)
    # ui_handles['control'] = create_ui_control(root, q)
    #ui_handles['logging'] = create_ui_logging(root, q)
    ui_handles['ros'] = create_ui_ros(root, q)
    ui_handles['process'] = create_ui_process(root, q)
    ui_handles['exe'] = create_ui_execution(root, q)
    return ui_handles


def create_ui_ros(root, q: Queue):

    ui_handles = {}

    # Title and frame
    title = tk.Label(root, text="ROS Connection")
    title.pack(anchor=tk.NW, expand=0, side=tk.TOP, padx=3, pady=3)
    frame = ttk.Frame(root, borderwidth=2, relief='solid')
    frame.pack(fill=tk.BOTH, expand=0, side=tk.TOP, padx=6, pady=3)

    # Button Handle
    def on_ros_connect_button_click(event=None):
        logger_ui.info("Button Pressed: Connect to ROS")
        ip = ros_ip_entrybox.get()
        q.put(SimpleNamespace(type=BackgroundCommand.UI_ROS_CONNECT, ip=ip))

    tk.Label(frame, text="ROS Core IP Address: ", font=tk.font_key,
             anchor=tk.SE).pack(side=tk.LEFT, fill=tk.Y, padx=10)
    ui_handles['ros_ip_entry'] = tk.StringVar(value="127.0.0.0")
    ros_ip_entrybox = tk.Entry(frame, textvariable=ui_handles['ros_ip_entry'])
    ros_ip_entrybox.pack(side=tk.LEFT)
    tk.Button(frame, text="Connect",
              command=on_ros_connect_button_click).pack(side=tk.LEFT)
    # Status Label
    tk.Label(frame, text="Status: ", font=tk.font_key,
             anchor=tk.SE).pack(side=tk.LEFT, fill=tk.Y, padx=10)
    ui_handles['ros_status'] = tk.StringVar(value="Not Connected")
    tk.Label(frame, textvariable=ui_handles['ros_status'], font=tk.font_key, anchor=tk.SE).pack(
        side=tk.LEFT, fill=tk.Y, padx=10)

    return ui_handles


def create_ui_process(root, q: Queue):

    ui_handles = {}

    # Title and frame
    title = tk.Label(root, text="Process JSON")
    title.pack(anchor=tk.NW, expand=0, side=tk.TOP, padx=3, pady=3)
    frame = ttk.Frame(root, borderwidth=2, relief='solid')
    frame.pack(fill=tk.BOTH, expand=0, side=tk.TOP, padx=6, pady=3)

    # Button Handle
    def on_load_process_button_click(event=None):
        logger_ui.info("Button Pressed: Load Json")
        filename = filedialog.askopenfilename(
            initialdir="/", title="Select file", filetypes=(("json files", "*.json"), ("all files", "*.*")))
        if filename == "":
            logger_ui.info("User canceled the Load Json File Dialog.")
        else:
            logger_ui.info(
                "User Selected %s from Load Json File Dialog." % filename)
            q.put(SimpleNamespace(
                type=BackgroundCommand.MODEL_LOAD_PROCESS, json_path=filename))

    tk.Label(frame, text="Process JSON: ", font=tk.font_key,
             anchor=tk.SE).pack(side=tk.LEFT, fill=tk.Y, padx=10)
    # Status Label
    ui_handles['process_status'] = tk.StringVar(value="Not Loaded")
    tk.Label(frame, textvariable=ui_handles['process_status'], font=tk.font_key, anchor=tk.SE).pack(
        side=tk.LEFT, fill=tk.Y, padx=10)
    # Load Button
    tk.Button(frame, text="Load Json File.",
              command=on_load_process_button_click).pack(side=tk.LEFT)

    # Second Frame holds the treeview for process Movements and Actions List
    frame = ttk.Frame(root, borderwidth=2, relief='solid')
    frame.pack(fill=tk.BOTH, expand=1, side=tk.TOP, padx=6, pady=3)
    tree = ttk.Treeview(frame, selectmode='browse')

    tree["columns"] = ("description", "details", "traj_points", "speed_type", "speed")
    tree.column("#0", width=150, minwidth=20, stretch=tk.NO)
    tree.column("description", width=180, minwidth=30, stretch=tk.NO)
    tree.column("details", width=200, minwidth=30)
    tree.column("traj_points", width=100, minwidth=20, stretch=tk.NO)
    tree.column("speed_type", width=100, minwidth=20, stretch=tk.NO)
    tree.column("speed", width=40, minwidth=20, stretch=tk.NO)

    tree.heading("#0", text="Name", anchor=tk.W)
    tree.heading("description", text="Description", anchor=tk.W)
    tree.heading("details", text="Details", anchor=tk.W)
    tree.heading("traj_points", text="TrajectoryPoints", anchor=tk.W)
    tree.heading("speed_type", text="Speed Type", anchor=tk.W)
    tree.heading("speed", text="mm/s", anchor=tk.W)

    tree.pack(fill=tk.BOTH, expand=1, padx=6, pady=3, side=tk.LEFT)

    # scrollbar
    vsb = ttk.Scrollbar(frame, orient="vertical", command=tree.yview)
    # vsb.place(relx=0.978, rely=0.175, relheight=0.713, relwidth=0.020)
    vsb.pack(side='right', fill='y')
    tree.configure(yscrollcommand=vsb.set)

    ui_handles['tree'] = tree
    return ui_handles


def init_actions_tree_view(guiref, model: RobotClampExecutionModel):
    """ Initialize the actions treeview after laoding process"""
    tree = guiref['process']['tree']  # type: ttk.Treeview
    process = model.process
    beam_item = None
    guiref['process']['item_ids'] = []  # type : List[str]
    for i, action in enumerate(process.actions):
        # Beam Row
        if isinstance(action, LoadBeamAction):
            beam_item = tree.insert(
                parent="", index="end", iid=action.beam_id, text="Beam %s" % action.beam_id, open=True)
            guiref['process']['item_ids'].append(action.beam_id)
        # Action Row
        action_item = tree.insert(parent=beam_item, index="end", iid=action.action_id, text="Action %i" % i,
                                  values=(action.__class__.__name__, "%s" % action, ""), open=True)
        guiref['process']['item_ids'].append(action.action_id)

        # Movement Rows
        for j, movement in enumerate(action.movements):
            description = movement.__class__.__name__
            details =  "%s" % movement
            traj_points = ""
            speed_type = movement.speed_type if hasattr(movement, 'speed_type') else ""
            speed = model.settings[speed_type] if hasattr(movement, 'speed_type') else ""
            movement_item = tree.insert(parent=action_item, index="end", iid=movement.move_id, text="Movement %i" % j,
                                        values=(description, details, traj_points, speed_type, speed))
            guiref['process']['item_ids'].append(movement.move_id)
            tree.see(movement_item)
            if tree.selection() == ():
                tree.selection_set(movement_item)

    tree.see(tree.selection())
    # Enable Run buttons
    guiref['exe']['run_button'].config(state="normal")
    guiref['exe']['step_button'].config(state="normal")
    guiref['exe']['stop_button'].config(state="normal")
    logger_ui.info("Actions Treeview Updated")
    


def treeview_get_selected_id(guiref):
    """Returns the currently selected id.
    Beaware it may not be a movement."""
    tree = guiref['process']['tree']  # type: ttk.Treeview
    move_id = tree.selection()[0]
    return move_id


def treeview_select_next_movement(guiref):
    """Select the next available movement"""

    tree = guiref['process']['tree']  # type: ttk.Treeview
    item_ids = guiref['process']['item_ids']
    index = item_ids.index(treeview_get_selected_id(guiref))
    if index < len(item_ids) - 1:
        new_selection = item_ids[index + 1]  # type: str
        tree.selection_set([new_selection])
        # Recursively call itself if the selected item is not a movement
        if not new_selection.startswith('m'):
            logger_ui.info("Skip selecting this movement: %s" % new_selection)
            treeview_select_next_movement(guiref)
        else:
            logger_ui.info("Next movement selected: %s" % new_selection)
            tree.see(new_selection)
    else:
        logger_ui.info("Cannot select next movement because end reached.")

    # Will return the selected move_id
    return treeview_get_selected_id(guiref)


def create_ui_execution(root, q: Queue):
    ui_handles = {}

    # Title and frame
    title = tk.Label(root, text="Execution / Run")
    title.pack(anchor=tk.NW, expand=0, side=tk.TOP, padx=3, pady=3)
    frame = ttk.Frame(root, borderwidth=2, relief='solid')
    frame.pack(fill=tk.BOTH, expand=0, side=tk.TOP, padx=6, pady=3)

    left_frame = ttk.Frame(frame, borderwidth=2, relief='solid', width=30)
    left_frame.pack(fill=tk.Y, expand=0, side=tk.LEFT, padx=6, pady=3)

    # Button Handle
    def on_run_button_click(event=None):
        logger_ui.info("Button Pressed: RUN")
        q.put(SimpleNamespace(type=BackgroundCommand.UI_RUN))
    ui_handles['run_button'] = tk.Button(
        left_frame, text="RUN", command=on_run_button_click, font=tk.big_button_font, width=20, state="disabled")
    ui_handles['run_button'].pack(side=tk.TOP)

    def on_step_button_click(event=None):
        logger_ui.info("Button Pressed: STEP")
        q.put(SimpleNamespace(type=BackgroundCommand.UI_STEP))
    ui_handles['step_button'] = tk.Button(
        left_frame, text="STEP", command=on_step_button_click, font=tk.big_button_font, width=20, state="disabled")
    ui_handles['step_button'].pack(side=tk.TOP)

    def on_stop_button_click(event=None):
        logger_ui.info("Button Pressed: STOP")
        q.put(SimpleNamespace(type=BackgroundCommand.UI_STOP))
    ui_handles['stop_button'] = tk.Button(
        left_frame, text="STOP", command=on_stop_button_click, font=tk.big_button_font, width=20, state="disabled")
    ui_handles['stop_button'].pack(side=tk.TOP)

    middle_frame = ttk.Frame(frame, borderwidth=2, relief='solid', width=400)
    middle_frame.pack(fill=tk.Y, expand=0, side=tk.LEFT, padx=6, pady=3)

    ui_handles['exe_status'] = tk.StringVar(value="Stopped")
    tk.Label(middle_frame, textvariable=ui_handles['exe_status'], font=tk.big_status_font, anchor=tk.CENTER, height=2).pack(
        side=tk.TOP, fill=tk.BOTH, padx=10)

    right_frame = ttk.Frame(frame, borderwidth=2, relief='solid', width=400)
    right_frame.pack(fill=tk.BOTH, expand=1, side=tk.LEFT, padx=6, pady=3)

    def on_confirm_button_click(event=None):
        logger_ui.info("Button Pressed: Confirm")
        q.put(SimpleNamespace(type=BackgroundCommand.UI_CONFIRM))
        confirm_button.config(state="disabled")

    ui_handles['confirm_button_text'] = tk.StringVar(value="Confirm?")
    ui_handles['confirm_button'] = confirm_button = tk.Button(middle_frame, textvariable=ui_handles['confirm_button_text'],
                                                              command=on_confirm_button_click, font=tk.big_button_font, width=20, height=2, state="disabled", bg='grey')
    ui_handles['confirm_button'].pack(side=tk.BOTTOM)

    return ui_handles


def ui_update_run_status(guiref, model: RobotClampExecutionModel):
    """Update the labels and indications related to execution"""
    if model.run_status == RunStatus.STOPPED:
        guiref['exe']['exe_status'].set("Stopped")
    if model.run_status == RunStatus.STEPPING_FORWARD:
        guiref['exe']['exe_status'].set("Stepping")
    if model.run_status == RunStatus.RUNNING:
        guiref['exe']['exe_status'].set("Running")
    if model.run_status == RunStatus.ERROR:
        guiref['exe']['exe_status'].set("Error Stopped")
