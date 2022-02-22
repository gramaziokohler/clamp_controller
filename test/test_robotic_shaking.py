import compas_rrc as rrc
from compas.geometry import Frame, Point, Vector
from compas_fab.backends.ros import RosClient
import time
from datetime import datetime
from copy import deepcopy

"""
This test is to see if I can overcome some situation when the robot is stuck trying to locak with the tool changer.
I can make the robot do some shaking.

To avoid runninng out of joint range or singularity, I will use the gantry to shake.
Acceleration is set to 100% for better shake effect.
Shake amount can be determined later

"""


if __name__ == '__main__':

    number_of_traj_points = 10

    ros = RosClient("192.168.0.120")
    ros.run()

    robot_11 = rrc.AbbClient(ros, '/rob1')

    print('Connected.')
    time.sleep(0.5)

    # # Get joints
    robot_joints, external_axes = robot_11.send_and_wait(rrc.GetJoints())
    robot_frame = robot_11.send_and_wait(rrc.GetFrame())

    # # Print received values
    print("robot_joints = %s, external_axes = %s" % (robot_joints, external_axes))
    print("robot_frame = %s" % robot_frame)

    # ! Generate Shake Trajectory
    shake_amount = 0.5  # mm
    shake_speed = 50
    e_pts = []

    for a in range(3):
        # Left
        e_pts.append(deepcopy(external_axes))
        e_pts[-1][a] += shake_amount
        # Right
        e_pts.append(deepcopy(external_axes))
        e_pts[-1][a] -= shake_amount
        # Center
        e_pts.append(deepcopy(external_axes))

    # j = deepcopy(robot_joints)
    # if j[3] > 0:
    #     final_joints_value[3] = final_joints_value[3] -100
    # else:
    #     final_joints_value[3] = final_joints_value[3] + 100

    # print("final_joints_value = %s" % final_joints_value)

    # robot_joint_targets = []
    # futures = []

    robot_11.send(rrc.CustomInstruction("r_A067_ActSoftMove", string_values=["XYRZ"], float_values=[5, 99]))
    robot_11.send(rrc.SetAcceleration(100, 100))
    # Send all points
    for ext_axes in e_pts:
        # Send robot command
        # robot_11.send(rrc.MoveToJoints(robot_joints, ext_axes, shake_speed, rrc.Zone.FINE))
        robot_11.send(rrc.MoveToJoints(robot_joints, ext_axes, shake_speed, rrc.Zone.Z0))

    robot_11.send_and_wait(rrc.CustomInstruction("r_A067_DeactSoftMove",  feedback_level=rrc.FeedbackLevel.DONE))
    robot_11.send(rrc.MoveToJoints(robot_joints, external_axes, shake_speed, rrc.Zone.FINE))
    print("Movement complete")

    time.sleep(3)
    ros.terminate()
    time.sleep(2)
    print('ros terminated')
