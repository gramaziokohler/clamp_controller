from robot_clamp_controller.ProcessModel import RobotClampExecutionModel, RunStatus
from integral_timber_joints.process.action import *
import time
import logging
import compas_rrc as rrc
from compas_fab.robots import to_degrees


logger_exe = logging.getLogger("app.exe")


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
    """Return True if the movement is executed to completion without problem."""
    # TODO IO pins are not calibrated.
    future_results = []  # type: List[rrc.FutureResult]
    if movement.digital_output == DigitalOutput.OpenGripper:
        future_results.append(model.ros_robot.send(
            rrc.SetDigital('doTc11Valve1A', 1, feedback_level=1)))
        future_results.append(model.ros_robot.send(
            rrc.SetDigital('doTc11Valve1B', 0, feedback_level=1)))

    if movement.digital_output == DigitalOutput.CloseGripper:
        future_results.append(model.ros_robot.send(
            rrc.SetDigital('doTc11Valve1A', 0, feedback_level=1)))
        future_results.append(model.ros_robot.send(
            rrc.SetDigital('doTc11Valve1B', 1, feedback_level=1)))

    if movement.digital_output == DigitalOutput.LockTool:
        future_results.append(model.ros_robot.send(
            rrc.CustomInstruction('r_Tc11_Lock', feedback_level=1)))

    if movement.digital_output == DigitalOutput.UnlockTool:
        future_results.append(model.ros_robot.send(
            rrc.CustomInstruction('r_Tc11_Unlock', feedback_level=1)))

    while (True):
        if all([future.done for future in future_results]):
            return True
        if model.run_status == RunStatus.STOPPED:
            logger_exe.warn(
                "Movement execution Stopped before completion (future not arrived): %s" % movement)
            return False
        time.sleep(0.05)

    return True

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

def execute_robotic_free_movement(model: RobotClampExecutionModel, movement: RoboticFreeMovement):

    for n, point in enumerate(movement.trajectory.points):
        assert len(point.values) == 8
        ext_values = to_millimeters(point.values[0:3])
        joint_values = to_degrees(point.values[3:10])
        zone = rrc.Zone.Z1
        speed = model.settings[movement.speed_type]
        model.ros_robot.send(rrc.MoveToJoints(model.joint_offset(joint_values), ext_values, speed, zone))

    return True

def execute_robotic_linaer_movement(model: RobotClampExecutionModel, movement: RoboticLinearMovement):
    return True

def execute_clamp_jaw_movement(model: RobotClampExecutionModel, movement: ClampsJawMovement):
    # model.ros_clamps.send_ROS_VEL_GOTO_COMMAND_wait()
    return True

def execute_some_delay(model: RobotClampExecutionModel, movement: Movement):
    for _ in range(10):
        time.sleep(0.3)
        if model.run_status == RunStatus.STOPPED:
            logger_exe.warn(
                "Movement execution Stopped before completion (future not arrived): %s" % movement)
            return False
    return True

def robot_goto_frame(model: RobotClampExecutionModel,  frame, speed):
    model.ros_robot.send_and_wait(rrc.MoveToFrame(frame, speed, rrc.Zone.FINE, motion_type=rrc.Motion.LINEAR))

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
