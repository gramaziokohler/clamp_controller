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

"""
This test is to see if I can overcome some situation when the robot is stuck trying to locak with the tool changer.
I can make the robot do some shaking.

To avoid runninng out of joint range or singularity, I will use the gantry to shake.
Acceleration is set to 100% for better shake effect.
Shake amount can be determined later

"""

def send_shake(robot, shake_amount, shake_speed, shake_repeat = 1):
    robot_joints, external_axes = robot.send_and_wait(rrc.GetJoints())
    robot_frame = robot.send_and_wait(rrc.GetFrame())
    logger.info("robot_joints = %s, external_axes = %s" % (robot_joints, external_axes))
    logger.info("robot_frame = %s" % robot_frame)
    robot.send(rrc.SetAcceleration(100, 100))
    e_pts = []
    for a in range(3):
        for r in range(shake_repeat):
            # Left
            e_pts.append(deepcopy(external_axes))
            e_pts[-1][a] += shake_amount
            # Right
            e_pts.append(deepcopy(external_axes))
            e_pts[-1][a] -= shake_amount
        # Center
        e_pts.append(deepcopy(external_axes))

    # Send all points
    for ext_axes in e_pts:
        # Send robot command
        # robot_11.send(rrc.MoveToJoints(robot_joints, ext_axes, shake_speed, rrc.Zone.FINE))
        robot.send(rrc.MoveToJoints(robot_joints, ext_axes, shake_speed, rrc.Zone.Z0))
    robot.send_and_wait(rrc.MoveToJoints(robot_joints, external_axes, shake_speed, rrc.Zone.FINE))
    logger.info("shake complete")


if __name__ == '__main__':

    number_of_traj_points = 10

    ros = RosClient("192.168.0.120")
    ros.run()

    robot = rrc.AbbClient(ros, '/rob1')

    print('Connected.')
    time.sleep(0.5)


    # ! Generate Shake Trajectory
    shake_amount = 1  # mm | min(0.3)
    shake_speed = 5 #mm/s | min(5 to 50)
    shake_repeat = 3

    # robot.send(rrc.CustomInstruction("r_A067_ActSoftMove", string_values=["XYRZ"], float_values=[5, 99]))

    send_shake(robot, shake_amount, shake_speed, shake_repeat)

    # robot.send_and_wait(rrc.CustomInstruction("r_A067_DeactSoftMove",  feedback_level=rrc.FeedbackLevel.DONE))

    print("Movement complete")

    time.sleep(3)
    ros.terminate()
    print('ros terminated')
