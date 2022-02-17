from robot_clamp_controller.GUI import *
from robot_clamp_controller.ProcessModel import *
from robot_clamp_controller.execute import robot_state_to_instruction, compute_visual_correction

# from integral_timber_joints.process.movement import *

class VisualOffsetPopup(object):

    def __init__(self, guiref,  model: RobotClampExecutionModel, movement: OperatorAddVisualOffset):
        self.window = tk.Toplevel(guiref['root'])
        self.guiref = guiref
        self.model = model
        self.movement = movement
        self.accpet = False

        tk.Label(self.window, text="Offset from the camera target (mm)?").grid(row=0, column=0, columnspan=3)

        # Entry Box for XYZ
        # self.offset_x = tk.StringVar(value="0")
        tk.Label(self.window, text="X").grid(row=1, column=0, columnspan=3)
        tk.Entry(self.window, textvariable=self.guiref['offset']['Visual_X']).grid(row=1, column=1, columnspan=3)
        # self.offset_y = tk.StringVar(value="0")
        tk.Label(self.window, text="Y").grid(row=2, column=0, columnspan=3)
        tk.Entry(self.window, textvariable=self.guiref['offset']['Visual_Y']).grid(row=2, column=1, columnspan=3)
        # self.offset_z = tk.StringVar(value="0")
        tk.Label(self.window, text="Z").grid(row=3, column=0, columnspan=3)
        tk.Entry(self.window, textvariable=self.guiref['offset']['Visual_Z']).grid(row=3, column=1, columnspan=3)

        # Buttons
        tk.Button(self.window, text='Go', command=self.go).grid(row=4, column=0)
        tk.Button(self.window, text='Accept', command=self.accept).grid(row=4, column=1)
        tk.Button(self.window, text='Cancel', command=self.cancel).grid(row=4, column=2)

        self.value = None

    def go(self):
        compute_visual_correction(self.guiref, self.model, self.movement)
        robot_config = self.movement.end_state['robot'].kinematic_config
        move_instruction = robot_state_to_instruction(self.guiref, self.model, robot_config, 30, rrc.Zone.FINE)
        self.model.ros_robot.send(move_instruction)

    def accept(self):
        compute_visual_correction(self.guiref, self.model, self.movement)
        self.accpet = True
        self.window.destroy()

    def cancel(self):
        self.model.run_status = RunStatus.STOPPED
        self.accpet = False
        self.window.destroy()

class MovementJsonPopup(object):

    def __init__(self, guiref,  model: RobotClampExecutionModel, movement: Movement):
        self.window = tk.Toplevel(guiref['root'])
        self.guiref = guiref
        self.model = model
        self.movement = movement

        tk.Label(self.window, text="Offset from the camera target (mm)?").grid(row=0, column=0)

        # Entry Box for XYZ
        # self.offset_x = tk.StringVar(value="0")
        t = tk.Text(self.window, height=200, width=250)
        t.grid(row=1, column=0)

        from compas.utilities import DataEncoder
        json_data = json.dumps(movement, indent=2, sort_keys=True, cls=DataEncoder)
        t.insert(tk.END, json_data)

        # Buttons
        tk.Button(self.window, text='Close', command=self.close).grid(row=2, column=0)

    def close(self):
        self.window.destroy()
