import compas_rrc as rrc
from compas.geometry import Frame, Point, Vector
from compas_fab.backends.ros import RosClient
import time
from datetime import datetime
from copy import deepcopy

"""
This test is to check the stall feedback mechanism for the Robot Controller:
- A rrc.AbbClient object is used to maintain the connection to the Robot Controller
- A number of points are sent to the Robot Controller in one go (5 pts, 20 pts)
- During movement, the test script will monitor the status of the robot using RemoteClampFunctionCall

- During movement, I will let go of the deadman switch
- During movement, I will press stop button
- During movement, I will intentionally collide with something

In all situations, the monitoring script need to raise a error message fast enough to indicate the stall of the clamp.

"""


def interpolate(start, end, total_steps, current_step):
    if current_step == 0:
        return start
    elif current_step == total_steps:
        return end
    elif (current_step > 0) & (current_step < total_steps):
        return [((e - s)/total_steps*current_step + s) for s, e in zip(start, end)]
    else:
        raise IndexError("current_step must be within 0 to total_steps")



def clear_robot_movement_buffers(robot: rrc.AbbClient):
    print("Flush begin")

    robot.send_and_wait(rrc.SystemSetDigital('do_A067_Interrupt', 1))
    while (True):
        result = robot.send_and_wait(rrc.SystemGetDigital('do_A067_InterruptDone'))
        print ("do_A067_InterruptDone = %s" % result)
        if result == 1.0:
            break
    robot.send(rrc.SetDigital('do_A067_Interrupt', 0))
    print("Flush complete")

def stop_robot(robot: rrc.AbbClient):
    robot.send_and_wait(rrc.SystemSetDigital('do_A067_Interrupt', 1))
    robot.send(rrc.SetDigital('do_A067_Interrupt', 0))


if __name__ == '__main__':

    number_of_traj_points = 10

    ros = RosClient("192.168.0.120")
    ros.run()

    robot_11 = rrc.AbbClient(ros, '/rob1')

    print('Connected.')
    time.sleep(0.5)
    # robot_11.send_and_wait(rrc.Noop())
    clear_robot_movement_buffers(robot_11)


    # # Get joints
    robot_joints, external_axes = robot_11.send_and_wait(rrc.GetJoints())
    robot_frame = robot_11.send_and_wait(rrc.GetFrame())

    # # Print received values
    print("robot_joints = %s, external_axes = %s" % (robot_joints, external_axes))
    print("robot_frame = %s" % robot_frame)

    # We rotate joint 4 only
    final_joints_value = deepcopy(robot_joints)
    if robot_joints[3] > 0:
        final_joints_value[3] = final_joints_value[3] -100
    else:
        final_joints_value[3] = final_joints_value[3] + 100

    print("final_joints_value = %s" % final_joints_value)

    robot_joint_targets = []
    futures = []



    # Send all points
    for current_step in range (0, number_of_traj_points):
        new_traj_point = interpolate(robot_joints, final_joints_value, number_of_traj_points - 1,  current_step)
        print (" - target : %s" % new_traj_point)
        robot_joint_targets.append(new_traj_point)

        # Send robot command
        # rrc.Zone.FINE and feedback_level=rrc.FeedbackLevel.DONE needed to have an accurate feedback timing.
        future = robot_11.send(rrc.MoveToJoints(new_traj_point, external_axes, 500, rrc.Zone.Z0, feedback_level=rrc.FeedbackLevel.DONE))
        futures.append(future)


    # Monitor the robot status
    active_point = 0
    last_time = datetime.now()
    while (True):
        if futures[active_point].done:
            print ("Future %i is done. Tool %f seconds." % (active_point, (datetime.now() - last_time).total_seconds()))
            last_time = datetime.now()
            active_point += 1

        if active_point >= len(futures):
            break

        task_excstate = robot_11.send_and_wait(rrc.GetTaskExecutionState('T_ROB11'))
        print ("task_excstate = %s" % task_excstate)
        if task_excstate != "started":
            print ("task_excstate not = started, Stopping Robot now.")
            stop_robot(robot_11)
            break

        time.sleep(0.2)

        # If I trigger the stop with code it is very fast reacting
        # If I trigger the stop with TP Stop button, the robot tend to continue for
        # two more seconds before stopping.
        #
        # if futures[5].done:
        #     stop_robot(robot_11)
        #     break



    print("Movement complete")


    time.sleep(3)
    ros.terminate()
    time.sleep(2)
    print('ros terminated')
