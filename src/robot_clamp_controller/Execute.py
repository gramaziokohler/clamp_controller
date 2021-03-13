from robot_clamp_controller.ProcessModel import RobotClampExecutionModel, RunStatus
from integral_timber_joints.process.action import *
import time
import logging
import compas_rrc as rrc
from compas_fab.robots import to_degrees
from compas_fab.robots.configuration import Configuration
import datetime


logger_exe = logging.getLogger("app.exe")

# Tool data settings in Robot Studio:
# TASK PERS tooldata t_A067_Tool:=[TRUE,[[0,0,0],[1,0,0,0]],[2.12,[0,0,45],[1,0,0,0],0,0,0]];
# TASK PERS tooldata t_A067_Tool_PG1000:=[TRUE,[[0,0,0],[1,0,0,0]],[10.72,[0,0,105],[1,0,0,0],0,0,0]];
# TASK PERS tooldata t_A067_Tool_PG500:=[TRUE,[[0,0,0],[1,0,0,0]],[9.04,[0,0,110],[1,0,0,0],0,0,0]];
# TASK PERS tooldata t_A067_Tool_CL3:=[TRUE,[[0,0,0],[1,0,0,0]],[7.99,[-30,4,73],[1,0,0,0],0,0,0]];


def current_milli_time(): return int(round(time.time() * 1000))


def execute_movement(guiref, model: RobotClampExecutionModel, movement: Movement):
    """Return True if the movement is executed to completion without problem.

    This is a blocking call that returns only if a movement is completed
    or if model.run_status == RunStatus.STOPPED. """

    logger_exe.info("Executing Movement (%s): %s" % (movement.movement_id, movement.tag))
    logger_exe.info(" - %s" % movement)

    guiref['exe']['last_executed_movement'].set(movement.movement_id)

    model.ros_robot.send(rrc.CustomInstruction(
        'r_A067_TPPlot', ["Executing Movement (%s)" % (movement.movement_id)], []))
    model.ros_robot.send(rrc.CustomInstruction(
        'r_A067_TPPlot', ["- %s" % (movement.tag[:40]) + '...'], []))

    if isinstance(movement, OperatorLoadBeamMovement):
        print("Operator should load beam %s. Grasp face is %s." %
              (movement.beam_id, movement.grasp_face))
        return True

    elif isinstance(movement, RoboticClampSyncLinearMovement):
        return execute_robotic_clamp_sync_linear_movement(guiref, model, movement)

    elif isinstance(movement, RoboticFreeMovement):
        return execute_robotic_free_movement(guiref, model, movement)

    elif isinstance(movement, RoboticLinearMovement):
        return execute_robotic_linaer_movement(guiref, model, movement)

    elif isinstance(movement, ClampsJawMovement):
        return execute_clamp_jaw_movement(guiref, model, movement)

    elif isinstance(movement, RoboticDigitalOutput):
        return execute_robotic_digital_output(guiref, model, movement)

    # Catch all during Development
    else:
        return execute_some_delay(model, movement)

#####################################################
# Sub functions that handels different Movement Types
#####################################################


def grip_load_instruction_from_beam(model: RobotClampExecutionModel, beam_id: str):
    beam = model.process.assembly.beam(beam_id)

    # rough eastimate of beam weight
    beam_volume = beam.width * beam.length * beam.height  # mm
    beam_density = 460  # kg/m^3
    beam_weight = beam_density * beam_volume * 1e-9

    # Estimate of beam cog
    gripper_grasp_dist_from_start = model.process.assembly.get_beam_attribute(beam_id, "gripper_grasp_dist_from_start")
    beam_grasp_offset = beam.length / 2 - gripper_grasp_dist_from_start

    # The mass (weight) of the load in kg.
    # Must be bigger then 0
    mass = beam_weight
    # The center of gravity of the payload expressed in mm in the tool coordinate system.
    # Minimum 1 value bigger then 0
    cog_x = beam_grasp_offset
    cog_y = 0
    cog_z = 241.6
    # The orientation of the axes of moment.
    # These are the principal axes of the payload moment of inertia with origin in center of gravity.
    # Expressed in quaternians
    aom_q1 = 1
    aom_q2 = 0
    aom_q3 = 0
    aom_q4 = 0
    # The moment of inertia of the load around the axis of moment expressed in kgm2.
    # Correct definition of the moments of inertia will allow optimal utilization of the path planner and axes control.
    # This may be of special importance when handling large sheets of metal, and so on.
    # All moments of inertia ix, iy, and iz equal to 0 kgm2 imply a point mass.
    # Normally, the moments of inertia must only be defined when the distance from the mounting flange to the center of gravity
    # is less than the maximal dimension of the load.
    inertia_x = 0
    inertia_y = 0
    inertia_z = 0
    logger_exe.info("Grip Load generated for beam %s, beam_weight = %skg, beam_grasp_offset = %smm" % (beam_id, beam_weight, beam_grasp_offset))
    return rrc.CustomInstruction('r_A067_GripLoad', [], [mass, cog_x, cog_y, cog_z, aom_q1, aom_q2, aom_q3, aom_q4, inertia_x, inertia_y, inertia_z], feedback_level=rrc.FeedbackLevel.DONE)


def execute_robotic_digital_output(guiref, model: RobotClampExecutionModel, movement: RoboticDigitalOutput):
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

        # Set Grip Load to zero
        future_results.append(model.ros_robot.send(
            rrc.CustomInstruction('r_A067_GripUnload', [], [], feedback_level=rrc.FeedbackLevel.DONE)))
        logger_exe.info("Gripper opening, grip load set to zero.")

    # Close Gripper Valve
    if movement.digital_output == DigitalOutput.CloseGripper:
        # Digital Output
        future_results.append(model.ros_robot.send(
            rrc.SetDigital('doUnitR11ValveA2', 1, feedback_level=1)))
        future_results.append(model.ros_robot.send(
            rrc.SetDigital('doUnitR11ValveB2', 0, feedback_level=1)))

        # Set Grip Load if the gripper closes on a beam
        if movement.beam_id is not None:
            future_results.append(model.ros_robot.send(
                grip_load_instruction_from_beam(model, movement.beam_id)))
            logger_exe.info("Closing Gripper %s on beam %s (new load set)" % (movement.tool_id, movement.beam_id))

    # Lock Tool Valve
    if movement.digital_output == DigitalOutput.LockTool:
        # Set tool data
        tool_data_name = 't_A067_Tool_' + model.process.tool(movement.tool_id).type_name
        logger_exe.info("Locking to tool %s (new tooldata = %s)" % (movement.tool_id, tool_data_name))

        future_results.append(model.ros_robot.send(
            rrc.SetTool(tool_data_name, feedback_level=rrc.FeedbackLevel.DONE)))
        # Digital Out
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

        # Set grip data to `zero` and tool data to `no tool`
        future_results.append(model.ros_robot.send(
            rrc.CustomInstruction('r_A067_GripUnload', [], [], feedback_level=rrc.FeedbackLevel.DONE)))
        tool_data_name = 't_A067_Tool'
        future_results.append(model.ros_robot.send(
            rrc.SetTool(tool_data_name, feedback_level=rrc.FeedbackLevel.DONE)))
        logger_exe.info("Unlocking tool %s (new tooldata = %s)" % (movement.tool_id, tool_data_name))

    # Add some delay after the action
    future_results.append(model.ros_robot.send(rrc.WaitTime(2, feedback_level=1)))

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


def execute_robotic_free_movement(guiref, model: RobotClampExecutionModel, movement: RoboticFreeMovement):
    """Performs RoboticFreeMovement Movement by setting the robot's IO signals

    This functions blocks and waits for the completion. For example if operator did not
    press Play on the robot contoller, this function will wait for it.
    Return True if the movement is executed to completion without problem.

    In case user wants to stop the wait, user can press the stop button
    on UI (sets the model.run_status to RunStatus.STOPPED) and this function
    will return False.
    """

    INTERMEDIATE_ZONE = rrc.Zone.Z5
    FINAL_ZONE = rrc.Zone.FINE
    STEPS_TO_BUFFER = 2  # Number of steps to allow in the robot buffer

    if movement.trajectory is None:
        logger_exe.warn("Attempt to execute movement with no trajectory")
        return False

    # We store the future results of the movement commands in this list.
    # This allow us to monitor the results and keep a known number of buffer points
    futures = []
    active_point = 0
    total_steps = len(movement.trajectory.points)
    last_time = datetime.datetime.now()

    # Check soft move state and send softmove command is state is different.
    # - the movement.softmove properity was marked by _mark_movements_as_softmove() when process is loaded.
    success = robot_softmove_blocking(model, enable=movement.softmove, soft_direction="XYZ",
                                      stiffness=99, stiffness_non_soft_dir=100)
    if not success:
        logger_exe.warn("execute_robotic_free_movement() stopped beacause robot_softmove_blocking() failed.")
        return False

    # In case of a START_FROM_PT
    if model.run_status == RunStatus.STEPPING_FORWARD_FROM_PT:
        active_point = model.alternative_start_point


    guiref['exe']['last_completed_trajectory_point'].set(" - ")
    guiref['exe']['last_deviation'].set(" - ")
    for current_step, point in enumerate(movement.trajectory.points):
        # Skip points in the case of a halfway start
        if current_step < active_point:
            # insert a dummy future to fill the gap
            futures.append(None)
            continue

        # Format movement and send robot command
        logger_exe.info("Sending command %i of %i" % (current_step, total_steps))
        assert len(point.values) == 9
        j = to_degrees(point.values[3:10])
        e = to_millimeters(point.values[0:3])

        speed = model.settings[movement.speed_type]
        # zone for intermediate vs final zone
        if current_step < total_steps - 1:
            zone = INTERMEDIATE_ZONE
        else:
            zone = FINAL_ZONE
        future = model.ros_robot.send(rrc.MoveToJoints(j, e, speed, zone, feedback_level=rrc.FeedbackLevel.DONE))
        futures.append(future)

        # Lopping while active_point is just STEPS_TO_BUFFER before the current_step.
        while (True):
            # Break the while loop and allow next point
            if active_point >= current_step - STEPS_TO_BUFFER:
                break
            # Advance pointer when future is done
            if futures[active_point].done:
                # Compute Deviation
                deviation = "?"
                logger_exe.info("Point %i is done. Delta time %f seconds. Deviation is %s" %
                                (active_point, (datetime.datetime.now() - last_time).total_seconds(), deviation))
                last_time = datetime.datetime.now()
                guiref['exe']['last_completed_trajectory_point'].set(str(active_point))
                guiref['exe']['last_deviation'].set(str(deviation))
                active_point += 1
            # Breaks entirely if model.run_status is STOPPED
            if model.run_status == RunStatus.STOPPED:
                logger_exe.warn("execute_robotic_free_movement stopped before completion (future not arrived)")
                return False

    # Report deviation
    actual_frame, ext_axes = model.ros_robot.send_and_wait(rrc.GetRobtarget())
    target_frame = movement.target_frame
    deviation = target_frame.point.distance_to_point(actual_frame.point)
    logger_exe.info("Movement (%s) target frame deviation %s mm" % (movement.movement_id, deviation))

    return True


def execute_robotic_linaer_movement(guiref, model: RobotClampExecutionModel, movement: RoboticLinearMovement):

    return execute_robotic_free_movement(guiref, model, movement)


def execute_robotic_clamp_sync_linear_movement(guiref, model: RobotClampExecutionModel, movement: RoboticClampSyncLinearMovement):
    """Performs RoboticFreeMovement Movement by setting the robot's IO signals

    This functions blocks and waits for the completion. For example if operator did not
    press Play on the robot contoller, this function will wait for it.
    Return True if the movement is executed to completion without problem.

    In case user wants to stop the wait, user can press the stop button
    on UI (sets the model.run_status to RunStatus.STOPPED) and this function
    will return False.
    """

    INTERMEDIATE_ZONE = rrc.Zone.Z5
    FINAL_ZONE = rrc.Zone.FINE
    STEPS_TO_BUFFER = 1  # Number of steps to allow in the robot buffer

    if movement.trajectory is None:
        logger_exe.warn("Attempt to execute movement with no trajectory")
        return False
    futures = []
    active_point = 0
    total_steps = len(movement.trajectory.points)
    last_time = datetime.datetime.now()

    # # Check soft move state and send softmove command is state is different.
    # # - the movement.softmove properity was marked by _mark_movements_as_softmove() when process is loaded.
    # success = robot_softmove_blocking(model, enable=movement.softmove, soft_direction="XYZ",
    #                                     stiffness=99, stiffness_non_soft_dir=100)
    # if not success:
    #     logger_exe.warn("execute_robotic_free_movement() stopped beacause robot_softmove_blocking() failed.")
    #     return False

    # Ask user to press play on TP
    model.ros_robot.send(rrc.CustomInstruction('r_A067_TPPlot', ['Press PLAY to execute RobClamp Sync Move, CHECK speed is 100pct']))
    model.ros_robot.send(rrc.Stop())
    model.ros_robot.send_and_wait(rrc.CustomInstruction('r_A067_TPPlot', ['RobClamp Sync Move begins']))

    # Remove clamp prefix and send command
    clamp_ids = [clamp_id[1:] for clamp_id in movement.clamp_ids]
    velocity = model.settings[movement.speed_type]
    position = movement.jaw_positions[0]
    sequence_id = model.ros_clamps.send_ROS_VEL_GOTO_COMMAND(clamp_ids, position, velocity)
    clamp_action_finished = False

    # Wait for clamp to ACK
    while (True):
        if model.ros_clamps.sent_messages_ack[sequence_id] == True:
            logger_exe.info("Clamp Jaw Movement (%s) with %s to %smm Started" % (movement.movement_id, clamp_ids, position))
            break

    guiref['exe']['last_completed_trajectory_point'].set(" - ")
    guiref['exe']['last_deviation'].set(" - ")
    # Execute robot trajectory step by step
    for current_step, point in enumerate(movement.trajectory.points):
        if current_step < active_point:
            # insert a dummy future to fill the gap
            futures.append(None)
            continue

        # Format movement and send robot command
        logger_exe.info("Sending command %i of %i" % (current_step, total_steps))
        assert len(point.values) == 9
        j = to_degrees(point.values[3:10])
        e = to_millimeters(point.values[0:3])

        speed = model.settings[movement.speed_type]
        # zone for intermediate vs final zone
        if current_step < total_steps - 1:
            zone = INTERMEDIATE_ZONE
        else:
            zone = FINAL_ZONE
        future = model.ros_robot.send(rrc.MoveToJoints(j, e, speed, zone, feedback_level=rrc.FeedbackLevel.DONE))
        futures.append(future)

        # Lopping while active_point is just STEPS_TO_BUFFER before the current_step.
        while (True):
            # Break the while loop and allow next point
            if active_point >= current_step - STEPS_TO_BUFFER:
                break

            # Advance pointer when future is done
            if futures[active_point].done:
                # Compute Deviation
                deviation = "?"
                logger_exe.info("Point %i is done. Delta time %f seconds. Deviation is %s" %
                                (active_point, (datetime.datetime.now() - last_time).total_seconds(), deviation))
                last_time = datetime.datetime.now()
                guiref['exe']['last_completed_trajectory_point'].set(str(active_point))
                guiref['exe']['last_deviation'].set(str(deviation))
                active_point += 1

            # Breaks entirely if model.run_status is STOPPED
            if model.run_status == RunStatus.STOPPED:
                model.ros_clamps.send_ROS_STOP_COMMAND(clamp_ids)
                logger_exe.warn("execute_robotic_free_movement stopped before completion (future not arrived)")
                return False

            # Check if clamps are running or not

            if (not clamp_action_finished) and (not model.ros_clamps.sync_move_inaction):
                if model.ros_clamps.last_command_success:
                    logger_exe.info("Clamp Jaw Movement (%s) completed." % movement.movement_id)
                    for clamp_id in clamp_ids:
                        logger_exe.info("Clamp %s status: %s" % (clamp_id, model.ros_clamps.clamps_status[clamp_id]))
                else:
                    # If clamp is jammed, stop the execution
                    logger_exe.info("Clamp Jaw Movement (%s) stopped or jammed." % movement.movement_id)
                    for clamp_id in clamp_ids:
                        logger_exe.info("Clamp %s status: %s" % (clamp_id, model.ros_clamps.clamps_status[clamp_id]))
                    model.ros_robot.send(rrc.CustomInstruction('r_A067_TPPlot', ['RobClamp Sync Move Stopped because clamps jammed.']))
                    return False

                clamp_action_finished = True  # What is left is for the robot to finish
    model.ros_robot.send(rrc.CustomInstruction('r_A067_TPPlot', ['RobClamp Sync Move Completed']))
    return True


def execute_clamp_jaw_movement(guiref, model: RobotClampExecutionModel, movement: ClampsJawMovement):
    if (model.ros_clamps is None) or not model.ros_clamps.is_connected:
        logger_exe.info(
            "Clamp movement cannot start because Clamp ROS is not connected")
        return False

    # Remove clamp prefix:
    clamp_ids = [clamp_id[1:] for clamp_id in movement.clamp_ids]
    velocity = model.settings[movement.speed_type]
    position = movement.jaw_positions[0]
    sequence_id = model.ros_clamps.send_ROS_VEL_GOTO_COMMAND(clamp_ids, position, velocity)

    # Wait for clamp to ACK
    while (True):
        if model.ros_clamps.sent_messages_ack[sequence_id] == True:
            logger_exe.info("Clamp Jaw Movement (%s) with %s to %smm Started" % (movement.movement_id, clamp_ids, position))
            break

    model.ros_clamps.sync_move_inaction = True
    model.ros_clamps.last_command_success = False
    # Wait for clamp to complete
    while (True):
        # Check if clamps are running or not
        if not model.ros_clamps.sync_move_inaction:
            if model.ros_clamps.last_command_success:
                logger_exe.info("Clamp Jaw Movement (%s) completed." % movement.movement_id)
                for clamp_id in clamp_ids:
                    logger_exe.info("Clamp %s status: %s" % (clamp_id, model.ros_clamps.clamps_status[clamp_id]))

                return True
            else:
                logger_exe.info("Clamp Jaw Movement (%s) stopped or jammed." % movement.movement_id)
                for clamp_id in clamp_ids:
                    logger_exe.info("Clamp %s status: %s" % (clamp_id, model.ros_clamps.clamps_status[clamp_id]))
                return False

        # Check if user stopped the clampping process
        if model.run_status == RunStatus.STOPPED:
            logger_exe.warn(
                "Clamp Jaw Movement (%s) stopped before completion." % movement.movement_id)
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


def robot_softmove(model: RobotClampExecutionModel, enable: bool, soft_direction="Z", stiffness=99, stiffness_non_soft_dir=100):
    """Non-blocking call to enable or disable soft move. Not cancelable.
    `soft_direction` modes available are "Z", "XY", "XYZ", "XYRZ"
    - use "XY" or "XYRZ" for pushing hard but allow deviation
    - use "Z" to avoid pushing hard but be accurate on other axis.

    `stiffness` in the specified direction, 0 is softest, 100 is stiffness
    `stiffness` in the other non specified direction, 0 is softest, 100 is stiffness

    future result is returned.
    """
    # model.ros_robot.send(rrc.SetTool('t_A067_T1_Gripper'))  # TODO: This should not be hard coded.
    if enable:
        future = model.ros_robot.send(rrc.CustomInstruction("r_A067_ActSoftMove",
                                                            string_values=[soft_direction],
                                                            float_values=[stiffness, stiffness_non_soft_dir], feedback_level=rrc.FeedbackLevel.DONE))
        logger_exe.info("robot_softmove(Enabled) command sent.")
    else:
        future = model.ros_robot.send(rrc.CustomInstruction(
            "r_A067_DeactSoftMove",  feedback_level=rrc.FeedbackLevel.DONE))
        logger_exe.info("robot_softmove(Disable) command sent.")

    return future


def robot_softmove_blocking(model: RobotClampExecutionModel, enable: bool, soft_direction="Z", stiffness=99, stiffness_non_soft_dir=100):
    """Blocking call to change robot's softmove state. It can be used to enable or disable the softmove.
    When success, the state is set to model.ros_robot_state_softmove_enabled

    If the current model.ros_robot_state_softmove_enabled is the same as the `enable` input,
    no command will be sent and will immediately return True.

    This functions blocks and waits for the completion. For example if operator did not
    press Play on the robot contoller, this function will wait for it.
    Return True if the command is executed to completion without problem.

    In case user wants to stop the wait, user can press the stop button
    on UI (sets the model.run_status to RunStatus.STOPPED) and this function
    will return False.
    """

    if (model.ros_robot_state_softmove_enabled is None) or (model.ros_robot_state_softmove_enabled != enable):

        result = robot_softmove(model, enable, soft_direction=soft_direction, stiffness=stiffness,
                                stiffness_non_soft_dir=stiffness_non_soft_dir)
        while (True):
            # Wait until softmove state is set.
            if result.done:
                logger_exe.info("Softmove is now %s" % ("Enabled" if enable else "Disabled"))
                model.ros_robot_state_softmove_enabled = enable
                return True
            # Stop waiting if model.run_status is STOPPED
            if model.run_status == RunStatus.STOPPED:
                logger_exe.warn(
                    "robot_softmove_blocking stopped before future.done arrived")
                return False

    else:
        logger_exe.debug("robot_softmove_blocking() skipped - current softmove state (%s) is the same." % model.ros_robot_state_softmove_enabled)
        return True


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
