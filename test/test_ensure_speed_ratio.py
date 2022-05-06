import compas_rrc as rrc
from compas.geometry import Frame, Point, Vector
from compas_fab.backends.ros import RosClient
import time
from datetime import datetime
from copy import deepcopy

from robot_clamp_controller.rrc_instructions import *

"""
This test is to implment a check to ensure controller is at 100% speed ratio

"""


if __name__ == '__main__':

    number_of_traj_points = 5

    ros = RosClient("192.168.0.120")
    ros.run()

    robot_11 = rrc.AbbClient(ros, '/rob1')

    print('Connected.')
    time.sleep(0.5)

    def ensure_speed_ratio(robot:rrc.AbbClient, target_speed_ratio:float = 100):
        while (True):
            speed_ratio = robot.send_and_wait(GetSpeedRatio(), 2)
            print("speed_ratio = %s" % (speed_ratio))
            if abs(speed_ratio - target_speed_ratio) < 0.1:
                break
            # Print Message to TP and ask for chaning the speed ratio.
            robot.send(rrc.CustomInstruction('r_A067_TPPlot', ['Change Speed Ratio to %i percent. Press PLAY to Continue.' % target_speed_ratio]))
            robot.send(rrc.Stop())
            robot.send_and_wait(rrc.WaitTime(0.1))

    ensure_speed_ratio(robot_11)

    time.sleep(3)
    ros.terminate()
    time.sleep(2)
    print('ros terminated')
