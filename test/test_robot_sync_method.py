import compas_rrc as rrc
from compas.geometry import Frame, Point, Vector
from compas_fab.backends.ros import RosClient
import time
from datetime import datetime

def interpolate(start, end, total_steps, current_step):
    if current_step == 0:
        return start
    elif current_step == total_steps:
        return end
    elif (current_step > 0) & (current_step < total_steps):
        return [((e - s)/total_steps*current_step + s) for s, e in zip(start, end)]
    else:
        raise IndexError("current_step must be within 0 to total_steps")


if __name__ == '__main__':

    ros = RosClient("192.168.0.117")
    ros.run()

    robot_11 = rrc.AbbClient(ros, '/robot_11')

    print('Connected.')
    time.sleep(0.5)

    # # Get joints
    robot_joints, external_axes = robot_11.send_and_wait(rrc.GetJoints())
    robot_frame = robot_11.send_and_wait(rrc.GetFrame())

    # # Print received values
    print(robot_joints, external_axes)
    print(robot_frame)

    # Start Pose End Pose
    j_start = [-135.78, -3.8, 0.99, 148.77, 60.91, 86.02]
    j_end = [-52.08, -37.71, 26.89, 210.88, 50.57, 300.1]
    e_start = [20718.48, -4152.34, -3349.22, 0.0, 0.0, 0.0]
    e_end = [20718.48, -4837.35, -3349.22, 0.0, 0.0, 0.0]



    # Go to start position
    robot_11.send(rrc.MoveToJoints(j_start, e_start, 2000, rrc.Zone.FINE))

    # We interpolate the joint values for running
    robot_11.send(rrc.PrintText('Press PLAY to execute'))
    robot_11.send_and_wait(rrc.Stop())

    # Looping through the rest of the niterpolated trajectory
    futures = []
    active_point = 0
    total_steps = 400
    last_time = datetime.now()
    buffering_steps = 3 # Number of steps to allow in the robot buffer

    for current_step in range(0, total_steps):
        # Lopping while active_point is just 1 before the current_step.
        while (True):
            # Break the while loop and allow next point
            if active_point >= current_step - buffering_steps:
                break
            # Advance pointer when future is done
            if futures[active_point].done:
                print ("Future %i is done. Tool %f seconds." % (active_point, (datetime.now() - last_time).total_seconds()))
                last_time = datetime.now()
                active_point += 1

        print ("Sending command %i" % current_step)
        j = interpolate(j_start, j_end, total_steps,  current_step + 1)
        e = interpolate(e_start, e_end, total_steps,  current_step + 1)
        # Send robot command
        future = robot_11.send(rrc.MoveToJoints(j, e, 500, rrc.Zone.Z5, feedback_level=rrc.FeedbackLevel.DONE))
        futures.append(future)

    robot_11.send(rrc.PrintText('Test Finished'))

    # Time necessary to send all remaining commands.
    time.sleep(3)
    ros.terminate()
    time.sleep(2)
    print('ros terminated')
