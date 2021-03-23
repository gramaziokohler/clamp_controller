import datetime
import logging
import time
from types import SimpleNamespace

import compas_rrc as rrc
from compas.geometry.primitives.frame import Frame
from compas.geometry.primitives.point import Point
from compas.geometry.primitives.vector import Vector
from compas_fab.robots import to_degrees
from compas_fab.robots.configuration import Configuration
from integral_timber_joints.process.action import *

from robot_clamp_controller.BackgroundCommand import *
from robot_clamp_controller.ProcessModel import *
from robot_clamp_controller.GUI import *

logger_exe = logging.getLogger("app.exe")

# Tool data settings in Robot Studio:
# TASK PERS tooldata t_A067_Tool:=[TRUE,[[0,0,0],[1,0,0,0]],[2.12,[0,0,45],[1,0,0,0],0,0,0]];
# TASK PERS tooldata t_A067_Tool_PG1000:=[TRUE,[[0,0,0],[1,0,0,0]],[10.72,[0,0,105],[1,0,0,0],0,0,0]];
# TASK PERS tooldata t_A067_Tool_PG500:=[TRUE,[[0,0,0],[1,0,0,0]],[9.04,[0,0,110],[1,0,0,0],0,0,0]];
# TASK PERS tooldata t_A067_Tool_CL3:=[TRUE,[[0,0,0],[1,0,0,0]],[7.99,[-30,4,73],[1,0,0,0],0,0,0]];
INTERMEDIATE_ZONE = rrc.Zone.Z5
FINAL_ZONE = rrc.Zone.FINE


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
        return execute_robotic_linear_movement(guiref, model, movement)

    elif isinstance(movement, ClampsJawMovement):
        return execute_clamp_jaw_movement(guiref, model, movement)

    elif isinstance(movement, RoboticDigitalOutput):
        return execute_robotic_digital_output(guiref, model, movement)

    elif isinstance(movement, OperatorAddJogOffset):
        return execute_operator_add_jog_offset_movement(guiref, model, movement)

    elif isinstance(movement, RemoveOperatorOffset):
        return execute_remove_operator_offset_movement(guiref, model, movement)

    elif isinstance(movement, OperatorAddVisualOffset):
        return execute_operator_add_visual_offset_movement(guiref, model, movement)

    # Catch all during Development
    else:
        execute_some_delay(model, movement)
        logger_exe.warn("execution code for %s does not exist." % movement)
        return False

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


def execute_jog_robot_to_state(guiref, model, robot_state: Configuration, message: str, q):
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
    q.put(SimpleNamespace(type=BackgroundCommand.UI_UPDATE_STATUS))
    point = robot_state['robot'].kinematic_config
    ext_values = to_millimeters(point.values[0:3])
    joint_values = to_degrees(point.values[3:10])

    # Apply Offsets
    ext_values = apply_ext_offsets(guiref, ext_values)
    joint_values = apply_joint_offsets(guiref, joint_values)

    if message != "":
        model.ros_robot.send(rrc.PrintText(message))

    instruction = rrc.MoveToJoints(joint_values, ext_values, 1000, rrc.Zone.FINE, feedback_level=rrc.FeedbackLevel.DONE)
    future = send_and_wait_unless_cancel(model, instruction)
    if future.done:
        logger_exe.info("execute_jog_robot_to_state complete")
        model.run_status = RunStatus.STOPPED
    else:
        logger_exe.warn("execute_jog_robot_to_state Stopped before completion (future not arrived)")

    # Trigger update Status
    q.put(SimpleNamespace(type=BackgroundCommand.UI_UPDATE_STATUS))
    return future.done


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
    position_readout_futures = []
    active_point = 0
    position_readout_point = 0
    total_steps = len(movement.trajectory.points)
    last_time = datetime.datetime.now()

    # Check soft move state and send softmove command is state is different.
    # - the movement.softmove properity was marked by _mark_movements_as_softmove() when process is loaded.
    if get_softness_enable(guiref):
        if not robot_softmove_blocking(model, enable=movement.softmove, soft_direction=get_soft_direction(guiref),
                                       stiffness=get_stiffness_soft_dir(guiref), stiffness_non_soft_dir=get_stiffness_nonsoft_dir(guiref)):
            logger_exe.warn("execute_robotic_free_movement() stopped beacause robot_softmove_blocking() failed.")
            return False

    # In case of a START_FROM_PT
    # The active point number is shifted
    if model.run_status == RunStatus.STEPPING_FORWARD_FROM_PT:
        active_point = model.alternative_start_point
        position_readout_point = active_point

    guiref['exe']['last_completed_trajectory_point'].set(" - ")
    guiref['exe']['last_deviation'].set(" - ")
    for current_step, point in enumerate(movement.trajectory.points):
        # Skip points in the case of a halfway start
        if current_step < active_point:
            # insert a dummy future to fill the gap
            futures.append(None)
            position_readout_futures.append(None)
            continue

        # Format movement and send robot command
        logger_exe.info("Sending command %i of %i" % (current_step, total_steps))
        instruction = trajectory_point_to_instruction(model, movement, guiref, current_step)
        futures.append(model.ros_robot.send(instruction))

        # Send command to read position back
        # position_readout_futures.append(model.ros_robot.send(rrc.GetRobtarget()))

        # Lopping while active_point is just STEPS_TO_BUFFER before the current_step.
        while (True):
            # Break the while loop and allow next point
            if active_point >= current_step - STEPS_TO_BUFFER:
                break
            # Advance pointer when future is done
            if futures[active_point].done:

                logger_exe.info("Point %i is done. Delta time %f seconds." %
                                (active_point, (datetime.datetime.now() - last_time).total_seconds()))
                last_time = datetime.datetime.now()
                guiref['exe']['last_completed_trajectory_point'].set(str(active_point))
                active_point += 1

            # Read the RobTarget as it comes back, compute deviation
            # if position_readout_futures[position_readout_point].done:
            #     if movement.path_from_link is not None:
            #         target_frame = movement.path_from_link["robot11_tool0"][position_readout_point]  # type: Frame
            #         target_frame = frame_to_millimeters(target_frame)
            #         target_ext_axes = to_millimeters(movement.trajectory.points[position_readout_point].values[0:3])
            #         actual_frame, actual_ext_axes = position_readout_futures[position_readout_point].value
            #         actual_ext_axes = actual_ext_axes.values[0:3]
            #         deviation_frame = target_frame.point.distance_to_point(actual_frame.point)
            #         deviation_ext_axes = Point(*target_ext_axes).distance_to_point(Point(*actual_ext_axes))
            #         logger_exe.info("Deviation for point %i: Frame Deviation = %.2fmm, ExtAxes Deviation = %.2fmm" %
            #                         (position_readout_point, deviation_frame, deviation_ext_axes))
            #         guiref['exe']['last_deviation'].set("%.2fmm" % (deviation_frame))
            #     position_readout_point += 1

            # Breaks entirely if model.run_status is STOPPED
            if model.run_status == RunStatus.STOPPED:
                logger_exe.warn("execute_robotic_free_movement stopped before completion (future not arrived)")
                return False

    # Final deviation
    deviation = check_deviation(model, movement.target_frame)
    if deviation is not None:
        logger_exe.info("Movement (%s) target frame deviation %s mm" % (movement.movement_id, deviation))
        guiref['exe']['last_deviation'].set("%.2fmm" % (deviation))
    else:
        logger_exe.warn("execute_robotic_free_movement stopped before deviation result (future not arrived)")
        return False

    return True


def execute_robotic_linear_movement(guiref, model: RobotClampExecutionModel, movement: RoboticLinearMovement):

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

    STEPS_TO_BUFFER = 1  # Number of steps to allow in the robot buffer

    if movement.trajectory is None:
        logger_exe.warn("Attempt to execute movement with no trajectory")
        return False
    futures = []
    active_point = 0
    total_steps = len(movement.trajectory.points)
    last_time = datetime.datetime.now()

    # Check soft move state and send softmove command is state is different.
    # - the movement.softmove properity was marked by _mark_movements_as_softmove() when process is loaded.
    if get_softness_enable(guiref):
        if not robot_softmove_blocking(model, enable=movement.softmove, soft_direction=get_soft_direction(guiref),
                                       stiffness=get_stiffness_soft_dir(guiref), stiffness_non_soft_dir=get_stiffness_nonsoft_dir(guiref)):
            logger_exe.warn("execute_robotic_clamp_sync_linear_movement() stopped beacause robot_softmove_blocking() failed.")
            return False

    # Ask user to press play on TP
    model.ros_robot.send(rrc.CustomInstruction('r_A067_TPPlot', ['Press PLAY to execute RobClamp Sync Move, CHECK speed is 100pct']))
    model.ros_robot.send(rrc.Stop())
    result = send_and_wait_unless_cancel(model, rrc.CustomInstruction('r_A067_TPPlot', ['RobClamp Sync Move begins']))
    if result.done == False:
        logger_exe.warn("execute_robotic_clamp_sync_linear_movement() stopped beacause user canceled while waiting for TP Press Play.")
        return False

    # Remove clamp prefix and send command
    clamp_ids = [clamp_id[1:] for clamp_id in movement.clamp_ids]
    velocity = model.settings[movement.speed_type]
    position = movement.jaw_positions[0]
    sequence_id = model.ros_clamps.send_ROS_VEL_GOTO_COMMAND(clamp_ids, position, velocity)
    clamp_action_finished = False

    # Wait for clamp to ACK
    while (True):
        if model.ros_clamps.sent_messages_ack[sequence_id] == True:
            logger_exe.info("RoboticClampSync Movement (%s) with %s to %smm Started" % (movement.movement_id, clamp_ids, position))
            break
        if model.run_status == RunStatus.STOPPED:
            logger_exe.warn("RoboticClampSync Movement (%s) stopped by user before clamp ACK." % (movement.movement_id))
            model.ros_clamps.send_ROS_STOP_COMMAND(clamp_ids)
            return False

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
        instruction = trajectory_point_to_instruction(model, movement, guiref, current_step)
        futures.append(model.ros_robot.send(instruction))

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
                logger_exe.warn("execute_robotic_clamp_sync_linear_movement stopped before completion (future not arrived)")
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

    # Final deviation
    deviation = check_deviation(model, movement.target_frame)
    if deviation is not None:
        logger_exe.info("Movement (%s) target frame deviation %s mm" % (movement.movement_id, deviation))
        guiref['exe']['last_deviation'].set("%.2fmm" % (deviation))
    else:
        logger_exe.warn("execute_robotic_clamp_sync_linear_movement stopped before deviation result (future not arrived)")
        return False
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
    model.ros_clamps.last_command_success = None
    sequence_id = model.ros_clamps.send_ROS_VEL_GOTO_COMMAND(clamp_ids, position, velocity)

    # Wait for clamp to ACK
    while (True):
        if model.ros_clamps.sent_messages_ack[sequence_id] == True:
            logger_exe.info("Clamp Jaw Movement (%s) with %s to %smm Started" % (movement.movement_id, clamp_ids, position))
            break
        # # Check if the the reply is negative
        # if model.ros_clamps.sync_move_inaction == False:
        #     if model.ros_clamps.last_command_success == False:
        #         logger_exe.warn("Clamp Jaw Movement (%s) stopped because clamp ACK is not success." % (movement.movement_id))
        #         return False
        # Check if user stopped the clampping process
        if model.run_status == RunStatus.STOPPED:
            logger_exe.warn("Clamp Jaw Movement (%s) stopped by user before clamp ACK." % (movement.movement_id))
            model.ros_clamps.send_ROS_STOP_COMMAND(clamp_ids)
            return False

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


def execute_operator_add_jog_offset_movement(guiref, model: RobotClampExecutionModel, movement: OperatorAddJogOffset):
    """Performs OperatorAddJogOffset Movement by stopping execution and ask user to jog (using the robot TP).
    Upon continuing program from robot TP, the difference between the robot flange frame and the original frame
    is used as grantry XYZ offset.

    A move is performed to make sure the offset is applied on external axis.
    The robot joints will go back to the previous joint values in movement.end_state['robot'].
    """
    # Read current robot external axis and joint values
    future = send_and_wait_unless_cancel(model, rrc.GetRobtarget())
    if future.done:
        starting_frame, original_ext_axes = future.value
        logger_exe.info("First Frame: %s, %s" % (starting_frame, original_ext_axes))
    else:
        logger_exe.warn("OperatorAddJogOffset Stopped before completion (First GetRobtarget future not arrived)")
        return False

    ############################################

    # Ask user to jog from TP and Stop Excution
    model.ros_robot.send(rrc.CustomInstruction('r_A067_TPPlot', ["Jog now to align. Press PLAY when finished"], []))
    future = send_and_wait_unless_cancel(model, rrc.Stop())
    if not future.done:
        logger_exe.warn("OperatorAddJogOffset Stopped before completion (rrc.Stop future not arrived)")
        return False

    ############################################

    # Upon restart, read the current robot frame.
    future = send_and_wait_unless_cancel(model, rrc.GetRobtarget())
    if future.done:
        new_frame, new_ext_axes = future.value
        logger_exe.info("Second Frame: %s, %s" % (new_frame, new_ext_axes))

    else:
        logger_exe.warn("OperatorAddJogOffset Stopped before completion (Second GetRobtarget future not arrived)")
        return False

    ############################################

    # Compute offset and apply to external axis.
    offset = Vector.from_start_end(movement.original_frame.point, new_frame.point)
    logger_exe.info("Offset from jogging = %s (amount = %4g)" % (offset, offset.length))

    guiref['offset']['Ext_X'].set("%.4g" % round(offset.x, 4))
    guiref['offset']['Ext_Y'].set("%.4g" % round(-1 * offset.y, 4))
    guiref['offset']['Ext_Z'].set("%.4g" % round(-1 * offset.z, 4))
    guiref['offset']['Rob_J1'].set("0")
    guiref['offset']['Rob_J2'].set("0")
    guiref['offset']['Rob_J3'].set("0")
    guiref['offset']['Rob_J4'].set("0")
    guiref['offset']['Rob_J5'].set("0")
    guiref['offset']['Rob_J6'].set("0")

    ext_axes_with_offset = apply_ext_offsets(guiref, original_ext_axes)

    # Movement to make sure joints have no offset
    # Ask user to jog from TP and Stop Excution
    model.ros_robot.send(rrc.CustomInstruction('r_A067_TPPlot', ["Offset registered: X %.3g Y %.3g Z %.3g." % (offset.x, offset.y, offset.z)], []))
    model.ros_robot.send(rrc.CustomInstruction('r_A067_TPPlot', ["Press PLAY to accept with a gantry move."], []))

    future = send_and_wait_unless_cancel(model, rrc.Stop())
    if not future.done:
        logger_exe.warn("OperatorAddJogOffset Stopped before completion (rrc.Stop future not arrived)")
        return False
    # future = send_and_wait_unless_cancel(model, rrc.MoveToJoints(movement.end_state['robot'], ext_axes_with_offset, 20, rrc.Zone.FINE, rrc.Motion.LINEAR))
    # future = send_and_wait_unless_cancel(model, rrc.MoveToRobtarget(movement.original_frame, ext_axes_with_offset, 20, rrc.Zone.FINE, rrc.Motion.LINEAR))
    # if future.done:
    #     model.ros_robot.send(rrc.CustomInstruction('r_A067_TPPlot', ["Gantry Move Complete"], []))
    # else:
    #     logger_exe.warn("OperatorAddJogOffset Stopped before completion (MoveToRobtarget future not arrived)")
    #     return False

    return True


def execute_remove_operator_offset_movement(guiref, model: RobotClampExecutionModel, movement: RemoveOperatorOffset):
    """Performs RemoveOperatorOffset Movement by cenceling all offset. """
    guiref['offset']['Ext_X'].set("0")
    guiref['offset']['Ext_Y'].set("0")
    guiref['offset']['Ext_Z'].set("0")
    guiref['offset']['Rob_J1'].set("0")
    guiref['offset']['Rob_J2'].set("0")
    guiref['offset']['Rob_J3'].set("0")
    guiref['offset']['Rob_J4'].set("0")
    guiref['offset']['Rob_J5'].set("0")
    guiref['offset']['Rob_J6'].set("0")
    logger_exe.info("Operator offset removed.")
    return True


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


def execute_operator_add_visual_offset_movement(guiref, model: RobotClampExecutionModel, movement: OperatorAddVisualOffset):
    """Performs OperatorAddVisualOffset by having an interactive dialog
    Allowing user to key in offset value in flange frame.
    The offset is applied to the gantry and """

    # Open a dialog and ask user to key in three offset values
    # Apply offset to flange frame offset, jog robot to the movement.end_state
    if not model.process.movement_has_end_robot_config(movement):
        logger_exe.warn("Error Attempt to execute OperatorAddVisualOffset but the movement end_state does not have robot config.")
        return False

    dialog = VisualOffsetPopup(guiref, model, movement)
    guiref['root'].wait_window(dialog.window)
    if dialog.accept:
        return True
    else:
        return False

    # Movement to go to


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
    instruction = rrc.MoveToFrame(frame, speed, rrc.Zone.FINE, motion_type=rrc.Motion.LINEAR)
    send_and_wait_unless_cancel(model, instruction)


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
        logger_exe.info("robot_softmove(Enabled) command sent: soft_direction = %s, stiffness = %i, stiffness_non_soft_dir = %i" %
                        (soft_direction, stiffness, stiffness_non_soft_dir))
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
        logger_exe.info("robot_softmove_blocking() skipped - current softmove state (%s) is the same." % model.ros_robot_state_softmove_enabled)
        return True

#########################################
# Visual Correction Helper Functions
#########################################


def compute_visual_correction(guiref, model: RobotClampExecutionModel, movement: RoboticMovement):
    """Compute the gantry offset from the visual offset in gui.
    The movement must have a target_frame"""
    align_X = guiref['offset']['Visual_X'].get()
    align_Y = guiref['offset']['Visual_Y'].get()
    align_Z = guiref['offset']['Visual_Z'].get()

    # Retrive the selected movement target frame
    current_movement_target_frame = movement.target_frame

    from compas.geometry.transformations.transformation import Transformation
    from compas.geometry.primitives.vector import Vector

    T = Transformation.from_frame(current_movement_target_frame)
    flange_vector = Vector(align_X, align_Y, align_Z)
    world_vector = flange_vector.transformed(T)
    guiref['offset']['Ext_X'].set("%.4g" % round(world_vector.x, 4))
    guiref['offset']['Ext_Y'].set("%.4g" % round(-1 * world_vector.y, 4))
    guiref['offset']['Ext_Z'].set("%.4g" % round(-1 * world_vector.z, 4))
    logger_model.info("Visual correction of Flange %s from Flange %s. Resulting in World %s" % (flange_vector, current_movement_target_frame, world_vector))


#########################
# rrc Helper Functions
#########################

def send_and_wait_unless_cancel(model: RobotClampExecutionModel, instruction: rrc.ROSmsg) -> rrc.FutureResult:
    """Send instruction and wait for feedback.

    This is a blocking call, it will return if the ABBClient
    send the requested feedback, or if model.run_status == RunStatus.STOPPED

    Returns the Future Result. If future.done == True, the result will contain the feedback.
    Otherwise the result is not received. Meaning the function returned from user cancel.

    Args:
        instruction: ROS Message representing the instruction to send.
        model: RobotClampExecutionModel where model.ros_robot contains a ABBClient
    """
    instruction.feedback_level = rrc.FeedbackLevel.DONE
    future = model.ros_robot.send(instruction)
    while (True):
        if future.done:
            return future
        if model.run_status == RunStatus.STOPPED:
            return future


def check_deviation(model: RobotClampExecutionModel, target_frame: Frame) -> Optional[float]:
    # Final deviation
    future = send_and_wait_unless_cancel(model, rrc.GetRobtarget())
    if future.done:
        actual_frame, ext_axes = future.value
        deviation = target_frame.point.distance_to_point(actual_frame.point)
        return deviation
    else:
        return None


def trajectory_point_to_instruction(model: RobotClampExecutionModel, movement: Movement, guiref, point_n: int,
                                    apply_offset: bool = True,
                                    fine_zone_points: int = 1,
                                    feedback_level: rrc.FeedbackLevel = rrc.FeedbackLevel.DONE,
                                    ) -> rrc.MoveToJoints:
    """Create a rrc.MoveToJoints instruction for a trajectory point
    taking into account of the point_n in determining speed and zone
    apply offset set in the gui if apply_offset is true"""

    # Retrive a specific point in the trajectory.
    point = movement.trajectory.points[point_n]

    # Speed and Zone
    speed = model.settings[movement.speed_type]
    if point_n < len(movement.trajectory.points) - fine_zone_points:
        zone = INTERMEDIATE_ZONE
    else:
        zone = FINAL_ZONE

    return robot_state_to_instruction(guiref, model, point, speed, zone, feedback_level)


def robot_state_to_instruction(guiref, model: RobotClampExecutionModel, robot_config: Configuration, speed, zone,
                               feedback_level: rrc.FeedbackLevel = rrc.FeedbackLevel.DONE,
                               apply_offset: bool = True):
    """Create rrc.MoveToJoints instruction from robot_state
    apply offset set in the gui if apply_offset is true"""
    assert len(robot_config.values) == 9

    # Ext Axis and Joint Values
    ext_values = to_millimeters(robot_config.values[0:3])
    joint_values = to_degrees(robot_config.values[3:10])

    # Apply Axis / Joints Offsets
    if apply_offset:
        ext_values = apply_ext_offsets(guiref, ext_values)
        joint_values = apply_joint_offsets(guiref, joint_values)

    # Command to Move
    return rrc.MoveToJoints(joint_values, ext_values, speed, zone, feedback_level=feedback_level)

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


def frame_to_millimeters(frame_in_meters: Frame):
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
    new_point = Point(frame_in_meters.point.x * 1000.0, frame_in_meters.point.y * 1000.0, frame_in_meters.point.z * 1000.0, )
    return Frame(new_point, frame_in_meters.xaxis, frame_in_meters.yaxis)
