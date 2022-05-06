import tkinter as tk
import compas_rrc as rrc
from compas.geometry import Frame, Point, Vector
from compas_fab.backends.ros import RosClient
import time
from datetime import datetime
from copy import deepcopy

import logging
logger = logging.getLogger("test")
logger.setLevel(logging.DEBUG)
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
logger.addHandler(ch)


class GantryShakePopupTest(object):

    def __init__(self, guiref):
        self.window = tk.Toplevel(guiref['root'], width=200)
        self.guiref = guiref
        # self.model = model

        tk.Label(self.window, text="Shake robot until ToolChanger Lock").grid(row=0, column=0, columnspan=2)

        # Entry Box for XYZ
        # self.offset_x = tk.StringVar(value="0")
        self.shake_amount = tk.DoubleVar(value=0.5)
        self.shake_speed = tk.DoubleVar(value=10)
        self.shake_repeat = tk.IntVar(value=1)

        tk.Label(self.window, text="Shake Amount").grid(row=1, column=0)
        tk.Scale(self.window, variable=self.shake_amount, from_=0.3, to_=3, orient=tk.HORIZONTAL, resolution=0.1, width=20, length=200).grid(row=1, column=1)
        tk.Label(self.window, text="Shake Speed mm / s").grid(row=2, column=0)
        tk.Scale(self.window, variable=self.shake_speed, from_=5, to_=30, orient=tk.HORIZONTAL, resolution=0.1, width=20, length=200).grid(row=2, column=1)
        tk.Label(self.window, text="Shake Repeat").grid(row=3, column=0)
        tk.Scale(self.window, variable=self.shake_repeat, from_=1, to_=3, orient=tk.HORIZONTAL, resolution=1, width=20, length=200).grid(row=3, column=1)

        tk.Label(self.window, text="Tool Changer Signal", height=5).grid(row=4, column=0)
        tk.Label(self.window, textvariable=self.guiref['status']).grid(row=4, column=1)

        # Buttons
        tk.Button(self.window, text='Shake', command=self.shake).grid(row=5, column=0)
        tk.Button(self.window, text='Cancel', command=self.cancel).grid(row=5, column=1)

        self.value = None

    def shake(self):
        pass

    def cancel(self):
        self.window.destroy()


if __name__ == '__main__':
    root = tk.Tk()
    guiref = {}
    guiref['root'] = root


    guiref['status'] = tk.StringVar(value=" ? ")

    def test_change_status():
        guiref['status'].set("Meow")

    tk.Button(root, text='Shake', command=test_change_status).grid(row=5, column=0)

    dialog = GantryShakePopupTest(guiref)
    tk.mainloop()