from robot_clamp_controller.ProcessModel import RobotClampExecutionModel, RunStatus
from integral_timber_joints.process.action import *
import time
import logging
import compas_rrc as rrc
from compas_fab.robots import to_degrees
from compas_fab.robots.configuration import Configuration
import datetime


logger_exe = logging.getLogger("app.exe")


def current_milli_time(): return int(round(time.time() * 1000))


def execute_movement(model: RobotClampExecutionModel, movement: Movement):
    """Return True if the movement is executed to completion without problem.

    This is a blocking call that returns only if a movement is completed
    or if model.run_status == RunStatus.STOPPED. """

    logger_exe.info("Executing Movement: %s" % movement)

    if isinstance(movement, OperatorLoadBeamMovement):
        print("Operator should load beam %s. Grasp face is %s." %
              (movement.beam_id, movement.grasp_face))
        return True

    elif isinstance(movement, RoboticClampSyncLinearMovement):
        return execute_robotic_clamp_sync_linear_movement(model, movement)

    elif isinstance(movement, RoboticFreeMovement):
        return execute_robotic_free_movement(model, movement)

    elif isinstance(movement, RoboticLinearMovement):
        return execute_robotic_linaer_movement(model, movement)

    elif isinstance(movement, ClampsJawMovement):
        return execute_clamp_jaw_movement(model, movement)

    elif isinstance(movement, RoboticDigitalOutput):
        return execute_robotic_digital_output(model, movement)

    # Catch all during Development
    else:
        return execute_some_delay(model, movement)

#####################################################
# Sub functions that handels different Movement Types
#####################################################


def execute_robotic_digital_output(model: RobotClampExecutionModel, movement: RoboticDigitalOutput):
    """Performs RoboticDigitalOutput Movement by setting the robot's IO signals

    This functions blocks and waits for the completion. For example if operator did not
    press Play on the robot contoller, this function will wait for it.
    Return True if the movement is executed to completion without problem.

    In case user wants to stop the wait, user can press the stop button
    on UI (sets the model.run_status to RunStatus.STOPPED) and this function
    will return False.
    """
    future_results = []  # type: List[rrc.FutureResult]
    # Open Gripper Valve
    if movement.digital_output == DigitalOutput.OpenGripper:
        future_results.append(model.ros_robot.send(
            rrc.SetDigital('doUnitR11ValveA2', 0, feedback_level=1)))
        future_results.append(model.ros_robot.send(
            rrc.SetDigital('doUnitR11ValveB2', 1, feedback_level=1)))

    # Close Gripper Valve
    if movement.digital_output == DigitalOutput.CloseGripper:
        future_results.append(model.ros_robot.send(
            rrc.SetDigital('doUnitR11ValveA2', 1, feedback_level=1)))
        future_results.append(model.ros_robot.send(
            rrc.SetDigital('doUnitR11ValveB2', 0, feedback_level=1)))

    # Lock Tool Valve
    if movement.digital_output == DigitalOutput.LockTool:
        future_results.append(model.ros_robot.send(
            rrc.SetDigital('doUnitR11ValveA1', 1, feedback_level=1)))
        future_results.append(model.ros_robot.send(
            rrc.SetDigital('doUnitR11ValveB1', 0, feedback_level=1)))

    # Unlock Tool Valve
    if movement.digital_output == DigitalOutput.UnlockTool:
        # Disconnect air supply to gripper feed through first
        future_results.append(model.ros_robot.send(
            rrc.SetDigital('doUnitR11ValveA2', 0, feedback_level=1)))
        future_results.append(model.ros_robot.send(
            rrc.SetDigital('doUnitR11ValveB2', 0, feedback_level=1)))
        # This is the valve for the tool changer
        future_results.append(model.ros_robot.send(
            rrc.SetDigital('doUnitR11ValveA1', 0, feedback_level=1)))
        future_results.append(model.ros_robot.send(
            rrc.SetDigital('doUnitR11ValveB1', 1, feedback_level=1)))

    while (True):
        if all([future.done for future in future_results]):
            return True
        if model.run_status == RunStatus.STOPPED:
            logger_exe.warn(
                "Movement execution Stopped before completion (future not arrived): %s" % movement)
            return False
        time.sleep(0.05)

    return True


def execute_jog_robot_to_state(model, robot_state: Configuration, message: str = ""):
    """Performs RoboticDigitalOutput Movement by setting the robot's IO signals

    This functions blocks and waits for the completion. For example if operator did not
    press Play on the robot contoller, this function will wait for it.
    Return True if the movement is executed to completion without problem.

    In case user wants to stop the wait, user can press the stop button
    on UI (sets the model.run_status to RunStatus.STOPPED) and this function
    will return False.
    """
    # Construct and send rrc command
    model.run_status = RunStatus.JOGGING

    point = robot_state['robot'].kinematic_config
    ext_values = to_millimeters(point.values[0:3])
    joint_values = to_degrees(point.values[3:10])

    if message != "":
        model.ros_robot.send(rrc.PrintText(message))
    future = model.ros_robot.send(rrc.MoveToJoints(
        joint_values, ext_values, 1000, rrc.Zone.FINE, feedback_level=rrc.FeedbackLevel.DONE))

    while (True):
        if future.done:
            logger_exe.info(
                "execute_jog_robot_to_state complete")
            return True
        if model.run_status == RunStatus.STOPPED:
            logger_exe.warn(
                "execute_jog_robot_to_state Stopped before completion (future not arrived)")
            return False


def execute_robotic_free_movement(model: RobotClampExecutionModel, movement: RoboticFreeMovement):
    """Performs RoboticFreeMovement Movement by setting the robot's IO signals

    This functions blocks and waits for the completion. For example if operator did not
    press Play on the robot contoller, this function will wait for it.
    Return True if the movement is executed to completion without problem.

    In case user wants to stop the wait, user can press the stop button
    on UI (sets the model.run_status to RunStatus.STOPPED) and this function
    will return False.
    """
    if movement.trajectory is None:
        logger_exe.warn("Attempt to execute movement with no trajectory")
        return False
    futures = []
    active_point = 0
    total_steps = len(movement.trajectory.points)
    last_time = datetime.datetime.now()
    buffering_steps = 2  # Number of steps to allow in the robot buffer

    for current_step, point in enumerate(movement.trajectory.points):

        # Lopping while active_point is just 1 before the current_step.
        while (True):
            # Break the while loop and allow next point
            if active_point >= current_step - buffering_steps:
                break
            # Advance pointer when future is done
            if futures[active_point].done:
                print("Point %i is done. Delta time %f seconds." %
                      (active_point, (datetime.datetime.now() - last_time).total_seconds()))
                last_time = datetime.datetime.now()
                active_point += 1
            # Breaks entirely if model.run_status is STOPPED
            if model.run_status == RunStatus.STOPPED:
                logger_exe.warn("execute_robotic_free_movement stopped before completion (future not arrived)")
                return False

        # Format and send robot command
        logger_exe.info("Sending command %i of %i" % (current_step, total_steps))
        assert len(point.values) == 9
        j = to_degrees(point.values[3:10])
        e = to_millimeters(point.values[0:3])
        speed = model.settings[movement.speed_type]
        zone = rrc.Zone.Z1
        future = model.ros_robot.send(rrc.MoveToJoints(j, e, speed, zone, feedback_level=rrc.FeedbackLevel.DONE))
        futures.append(future)

    return True


def execute_robotic_linaer_movement(model: RobotClampExecutionModel, movement: RoboticLinearMovement):

    return execute_robotic_free_movement(model, movement)


def execute_robotic_clamp_sync_linear_movement(model: RobotClampExecutionModel, movement: RoboticClampSyncLinearMovement):
    points = movement.trajectory.points
    points_sent = 0
    points_completed = 0
    # while(True):
    #     # Check if previously sent points (futures) is completed
    #     # Send points if the difference in two points are =< 1 and if we have more points left
    #
    #     pass
    return True


def execute_clamp_jaw_movement(model: RobotClampExecutionModel, movement: ClampsJawMovement):
    if (model.ros_clamps is None) or not model.ros_clamps.is_connected:
        logger_exe.info(
            "Clamp movement cannot start because Clamp ROS is not connected")
        return False
    # Remove clamp prefix:
    clamp_ids = [clamp_id[1:] for clamp_id in movement.clamp_ids]
    velocity = model.settings[movement.speed_type]
    sequence_id = model.ros_clamps.send_ROS_VEL_GOTO_COMMAND(
        clamp_ids, movement.jaw_positions[0], velocity)
    logger_exe.info("Clamp Jaw Movement Started: %s, seq_id = %s" %
                    (movement, sequence_id))
    while (True):
        if model.ros_clamps.sent_messages_ack[sequence_id] == True:
            logger_exe.info("Clamp Jaw Movement completed")
            return True
        if model.run_status == RunStatus.STOPPED:
            logger_exe.warn(
                "Movement execution Stopped before sequence_id (%s) is confirmed." % sequence_id)
            model.ros_clamps.send_ROS_STOP_COMMAND(clamp_ids)
            return False


def execute_some_delay(model: RobotClampExecutionModel, movement: Movement):
    for _ in range(10):
        time.sleep(0.3)
        if model.run_status == RunStatus.STOPPED:
            logger_exe.warn(
                "Movement execution Stopped before completion (future not arrived): %s" % movement)
            return False
    return True


def robot_goto_frame(model: RobotClampExecutionModel,  frame, speed):
    """Blocking call to go to a frame. Not cancelable."""
    model.ros_robot.send_and_wait(rrc.MoveToFrame(
        frame, speed, rrc.Zone.FINE, motion_type=rrc.Motion.LINEAR))


def robot_softmode(model: RobotClampExecutionModel, enable : bool, soft_direction = "Z", stiffness = 50, stiffness_non_soft_dir = 100):
    """Non-blocking call to enable or disable soft move. Not cancelable.
    `soft_direction` modes available are "Z", "XY", "XYZ", "XYRZ"
    - use "XY" or "XYRZ" for pushing hard but allow deviation
    - use "Z" to avoid pushing hard but be accurate on other axis.

    `stiffness` in the specified direction, 0 is softest, 100 is stiffness
    `stiffness` in the other non specified direction, 0 is softest, 100 is stiffness

    future result is returned.
    """
    model.ros_robot.send(rrc.SetTool('t_A067_T1_Gripper')) # TODO: This should not be hard coded.
    if enable:
        future = model.ros_robot.send(rrc.CustomInstruction("r_A067_ActSoftMove",
            string_values=[soft_direction],
            float_values=[stiffness, stiffness_non_soft_dir], feedback_level=rrc.FeedbackLevel.DONE))
        logger_exe.info("robot_softmode Enabled")
    else:
        future = model.ros_robot.send(rrc.CustomInstruction("r_A067_DeactSoftMove",  feedback_level=rrc.FeedbackLevel.DONE))
        logger_exe.info("robot_softmode Disabled")

    return future
##################
# Helper Functions
##################


def to_millimeters(meters):
    """Convert a list of floats representing meters to a list of millimeters.

    Parameters
    ----------
    radians : :obj:`list` of :obj:`float`
        List of distance values in meters.

    Returns
    -------
    :obj:`list` of :obj:`float`
        List of millimeters.
    """
    return [m * 1000.0 for m in meters]
