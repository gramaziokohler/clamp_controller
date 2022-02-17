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
from compas_fab.backends.ros.messages import ROSmsg
from integral_timber_joints.process.action import *

from robot_clamp_controller.background_command import *
from robot_clamp_controller.ProcessModel import *
from robot_clamp_controller.GUI import *
from robot_clamp_controller.rrc_instructions import *
from robot_clamp_controller.execute_popup import *

from clamp_controller.ScrewdriverModel import g_status_dict

logger_exe = logging.getLogger("app.exe")

"""
This file contains the procedural routines that is used to execute a movement.
Different Movement class are handeled by different `execute_` functions.
Connections to the various drivers are located in `RobotClampExecutionModel`.
- Calls are made to the Robot Controller via the `AbbClient` from the compas_rrc library. (`model.ros_robot`)
- Calls to the Remote Tools Controller is via the `RemoteClampFunctionCall` (`model.ros_clamps`)
- (Both objects are connected via ROS, however other connection objects can also be added if needed)

Many of the `execute_` functions are stoppable from the UI. This is used for any motion that may take a long
time to accomplish or may fail in an unexpected way. Therefor all functions that implements a waiting loop should
repeatedly check if `model.run_status == RunStatus.STOPPED`.

The `execute_` functions should return True if the procedure is executed successfully, and false if otherwise.
In case of execution failure, the `execute_` function is responsible for putting the system in a safe (stopped) state
before returning.

"""
# Tool data settings in Robot Studio:
# TASK PERS tooldata t_A067_Tool:=[TRUE,[[0,0,0],[1,0,0,0]],[2.12,[0,0,45],[1,0,0,0],0,0,0]];
# TASK PERS tooldata t_A067_Tool_PG1000:=[TRUE,[[0,0,0],[1,0,0,0]],[10.72,[0,0,105],[1,0,0,0],0,0,0]];
# TASK PERS tooldata t_A067_Tool_PG500:=[TRUE,[[0,0,0],[1,0,0,0]],[9.04,[0,0,110],[1,0,0,0],0,0,0]];
# TASK PERS tooldata t_A067_Tool_CL3:=[TRUE,[[0,0,0],[1,0,0,0]],[7.99,[-30,4,73],[1,0,0,0],0,0,0]];
INTERMEDIATE_ZONE = rrc.Zone.Z5
FINAL_ZONE = rrc.Zone.FINE


def current_milli_time(): return int(round(time.time() * 1000))


def execute_movement(guiref, model: RobotClampExecutionModel, movement: Movement):
    """Executes a Movement.
    This function is a switch board deligating the actual implementation to one of the
    `execute_` functions below.
    This function will return True if the movement is executed to completion without problem.

    This funcion will block until the movement is completed or if it is canceled from the UI.
    (`model.run_status == RunStatus.STOPPED.`)
    """

    logger_exe.info("Executing %s (%s): %s" % (movement, movement.movement_id, movement.tag))

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

    elif isinstance(movement, RobotScrewdriverSyncLinearMovement):
        return execute_robotic_clamp_sync_linear_movement(guiref, model, movement)

    elif isinstance(movement, RoboticFreeMovement):
        return execute_robotic_free_movement(guiref, model, movement)

    elif isinstance(movement, RoboticLinearMovement):
        return execute_robotic_linear_movement(guiref, model, movement)

    elif isinstance(movement, ClampsJawMovement):
        return execute_clamp_jaw_movement(guiref, model, movement)

    elif isinstance(movement, RoboticDigitalOutput):
        tool_id = movement.tool_id  # type: str
        tool_type = type(model.process.tool(tool_id))
        digital_output = movement.digital_output
        if (digital_output == DigitalOutput.OpenGripper or digital_output == DigitalOutput.CloseGripper) and tool_type == Screwdriver:
            return execute_robotic_digital_output_screwdriver(guiref, model, movement)
        else:
            return execute_robotic_digital_output(guiref, model, movement)

    elif isinstance(movement, OperatorAttachToolMovement):
        return execute_operator_attach_tool_movement(guiref, model, movement)

    elif isinstance(movement, OperatorAddJogOffset):
        return execute_operator_add_jog_offset_movement(guiref, model, movement)

    elif isinstance(movement, AcquireDockingOffset):
        return execute_acquire_docking_offset(guiref, model, movement)

    elif isinstance(movement, OperatorAddVisualOffset):
        return execute_operator_add_visual_offset_movement(guiref, model, movement)

    elif isinstance(movement, RemoveOperatorOffset):
        return execute_remove_operator_offset_movement(guiref, model, movement)

    elif isinstance(movement, CancelRobotOffset):
        return execute_remove_operator_offset_movement(guiref, model, movement)

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

    def wait_for_digital_signal(io_name, signal_value_to_wait_for):
        while (True):
            future = send_and_wait_unless_cancel(model, rrc.ReadDigital(io_name))
            # User pressed cancel
            if not future.done:
                return False
            # Check signal, if it is equal to expected value, we return
            if future.value == signal_value_to_wait_for:
                return True
            # If not, we send another read signal and check again

    while (True):
        # * Check for IO completion
        if all([future.done for future in future_results]):
            if movement.digital_output == DigitalOutput.LockTool:
                # Assert ToolChanger Lock Check signal is HIGH (1)
                if not wait_for_digital_signal('diUnitR11In3', 1):
                    logger_exe.warn("UI stop button pressed before getting confirm signal.")
                    return False
                else:
                    logger_exe.info("Toolchanger sensor signal received. DigitalOutput.LockTool is successful")
                    time.sleep(0.3)
                    return True

            if movement.digital_output == DigitalOutput.UnlockTool:
                # Assert ToolChanger Lock Check signal is HIGH (1)
                if not wait_for_digital_signal('diUnitR11In3', 0):
                    logger_exe.warn("UI stop button pressed before getting confirm signal.")
                    return False
                else:
                    logger_exe.info("Toolchanger sensor signal received. DigitalOutput.UnlockTool is successful")
                    time.sleep(0.3)
                    return True

            else:
                # Add some delay after the action and assume it is done
                future = send_and_wait_unless_cancel(model, rrc.WaitTime(2))
                if not future.done:
                    logger_exe.warn("UI stop button pressed before WaitTime(2) is over.")
                    return False
                else:
                    return True
        if model.run_status == RunStatus.STOPPED:
            logger_exe.warn(
                "UI stop button pressed before RoboticDigitalOutput (%s) is completed." % movement)
            return False
        time.sleep(0.05)


def execute_robotic_digital_output_screwdriver(guiref, model: RobotClampExecutionModel, movement: RoboticDigitalOutput):
    if (model.ros_clamps is None) or not model.ros_clamps.is_connected:
        logger_exe.info(
            "Screwdriver gripper movement cannot start because Clamp ROS is not connected")
        return False

    # Ping Screwdriver Controller and get a status update\
    model.ros_clamps.send_ros_command("ROS_REQUEST_STATUSUPDATE", None)
    timeout_threshold = 2
    start = time.time()
    model.ros_clamps.clamps_status = {}
    while(time.time() < start + timeout_threshold):
        if model.ros_clamps.clamps_status != {}:
            break
    if model.ros_clamps.clamps_status == {}:
        logger_exe.error("Screwdriver Controller did not respond with status update request.")
        return False

    # Remove clamp prefix:
    model.ros_clamps.last_command_success = None
    if movement.digital_output == DigitalOutput.OpenGripper:
        sequence_id = model.ros_clamps.send_ROS_GRIPPER_OPEN_COMMAND(movement.tool_id)
        model.ros_clamps.clamps_status[movement.tool_id]['raw_gripper_status'] = 2
        logger_exe.info("Sending send_ROS_GRIPPER_OPEN_COMMAND Movement (%s)" % (movement.movement_id))
    elif movement.digital_output == DigitalOutput.CloseGripper:
        sequence_id = model.ros_clamps.send_ROS_GRIPPER_CLOSE_COMMAND(movement.tool_id)
        logger_exe.info("Sending send_ROS_GRIPPER_CLOSE_COMMAND Movement (%s)" % (movement.movement_id))
        model.ros_clamps.clamps_status[movement.tool_id]['raw_gripper_status'] = 1
    else:
        logger_exe.error("Screwdriver Movement (%s) not supported for digital_output type: %s" % (movement.movement_id, movement.digital_output))
        return False

    # Wait for Clamp Controller to ACK
    while (True):
        if model.ros_clamps.sent_messages_ack[sequence_id] == True:
            logger_exe.info("Screwdriver Gripper Movement (%s) Started" % (movement.movement_id))
            break

        if model.ros_clamps.last_command_success == False:
            logger_exe.warn("Screwdriver Gripper Movement (%s) refused by tool controller." % (movement.movement_id))
            return False

        # Check if user pressed stop button in UI
        if model.run_status == RunStatus.STOPPED:
            logger_exe.warn("Screwdriver Gripper Movement (%s) stopped by user before clamp ACK." % (movement.movement_id))
            model.ros_clamps.send_ROS_STOP_COMMAND([movement.tool_id])
            return False

    model.ros_clamps.last_command_success = False

    # Wait for screwdriver to complete
    while (True):
        # Check if clamps are running or not
        # g_status 1 or 2 is moving, 3 or 4 is success, 5 or 6 is fail
        raw_gripper_status = model.ros_clamps.clamps_status[movement.tool_id]['raw_gripper_status']
        if raw_gripper_status not in [1, 2]:
            if raw_gripper_status in [3, 4]:
                logger_exe.info("Screwdriver Gripper Movement (%s) completed." % movement.movement_id)
                logger_exe.info("Screwdriver %s status: %s" % (movement.tool_id, model.ros_clamps.clamps_status[movement.tool_id]))
                model.ros_clamps.last_command_success = True
                return True
            else:
                logger_exe.info("Screwdriver Gripper Movement (%s) stopped or jammed." % movement.movement_id)
                logger_exe.info("Screwdriver %s status: %s" % (movement.tool_id, model.ros_clamps.clamps_status[movement.tool_id]))
                return False

        # Check if user pressed stop button in UI
        if model.run_status == RunStatus.STOPPED:
            logger_exe.warn(
                "UI stop button pressed before Screwdriver Gripper Movement (%s) is completed." % movement.movement_id)
            model.ros_clamps.send_ROS_STOP_COMMAND(movement.tool_id)
            return False


def execute_operator_attach_tool_movement(guiref, model: RobotClampExecutionModel, movement: RoboticDigitalOutput):
    robotic_digital_output_movement = RoboticDigitalOutput(DigitalOutput.CloseGripper, movement.tool_id, tag=movement.tag)
    robotic_digital_output_movement.movement_id = movement.movement_id
    return execute_robotic_digital_output_screwdriver(guiref, model, robotic_digital_output_movement)


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

    # Check softmove state and send softmove command if state is different.
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
            #         target_ext_axes = to_millimeters(movement.trajectory.points[position_readout_point].prismatic_values)
            #         actual_frame, actual_ext_axes = position_readout_futures[position_readout_point].value
            #         actual_ext_axes = actual_ext_axes.prismatic_values
            #         deviation_frame = target_frame.point.distance_to_point(actual_frame.point)
            #         deviation_ext_axes = Point(*target_ext_axes).distance_to_point(Point(*actual_ext_axes))
            #         logger_exe.info("Deviation for point %i: Frame Deviation = %.2fmm, ExtAxes Deviation = %.2fmm" %
            #                         (position_readout_point, deviation_frame, deviation_ext_axes))
            #         guiref['exe']['last_deviation'].set("%.2fmm" % (deviation_frame))
            #     position_readout_point += 1

            # Breaks entirely if model.run_status is STOPPED
            if model.run_status == RunStatus.STOPPED:
                logger_exe.warn("UI stop button pressed before Robotic Free Movement (%s) is completed." % (movement.movement_id))
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
    """Performs RoboticClampSyncLinearMovement Movement

    Syncronization
    --------------
    1. User Press Play on TP
    2. Send movement command to Clamp Controller (wait for message ACK)
    3. Send movement to Robot Controller (no wait)



    Execution blocking behaviour
    ----------------------------
    This functions blocks and waits for the completion. For example if operator did not
    press Play on the robot contoller, this function will wait for it.
    Return True if the movement is executed to completion without problem.

    Cancel
    ------
    In case user wants to stop the wait, user can press the stop button
    on UI (sets the model.run_status to RunStatus.STOPPED) and this function
    will give up, stop robot and clamps, and return False.
    """

    STEPS_TO_BUFFER = 1  # Number of steps to allow in the robot buffer

    if movement.trajectory is None:
        logger_exe.warn("Attempt to execute movement with no trajectory")
        return False
    futures = []
    active_point = 0
    total_steps = len(movement.trajectory.points)
    last_time = datetime.datetime.now()

    # * Prepare Movement command parameters
    if type(movement) == RoboticClampSyncLinearMovement:
        clamp_ids = movement.clamp_ids
        position = movement.jaw_positions[0]
    elif type(movement) == RobotScrewdriverSyncLinearMovement:
        clamp_ids = movement.screwdriver_ids
        position = movement.screw_positions[0]

    velocity = model.settings[movement.speed_type]

    # * Check softmove state and send softmove command if state is different.
    # - the movement.softmove properity was marked by _mark_movements_as_softmove() when process is loaded.
    if get_softness_enable(guiref):
        if not robot_softmove_blocking(model, enable=movement.softmove, soft_direction=get_soft_direction(guiref),
                                       stiffness=get_stiffness_soft_dir(guiref), stiffness_non_soft_dir=get_stiffness_nonsoft_dir(guiref)):
            logger_exe.warn("execute_robotic_clamp_sync_linear_movement() stopped beacause robot_softmove_blocking() failed.")
            return False

    # * Check to ensures Speed Ratio is 100 %
    if not ensure_speed_ratio(model, model.ros_robot, 100):
        return False

    # * User Press PLAY to Sart the Movement
    model.ros_robot.send(rrc.CustomInstruction('r_A067_TPPlot', ['Press PLAY to execute RobClamp Sync Move, CHECK speed is 100pct']))
    result = send_and_wait_unless_cancel(model, rrc.CustomInstruction('r_A067_TPPlot', ['RobClamp Sync Move begins']))
    if result.done == False:
        logger_exe.warn("execute_robotic_clamp_sync_linear_movement() stopped beacause user canceled while waiting for TP Press Play.")
        return False



    # Send movement to Clamp Controller, wait for ROS controller to ACK
    sequence_id = model.ros_clamps.send_ROS_VEL_GOTO_COMMAND(clamp_ids, position, velocity)
    clamp_action_finished = False
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
                logger_exe.warn("UI stop button pressed before Robotic Clamp Sync Linear Movement (%s) is completed." % (movement.movement_id))
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

    # * Final deviation
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
    clamp_ids = movement.clamp_ids
    velocity = model.settings[movement.speed_type]
    position = movement.jaw_positions[0]
    model.ros_clamps.last_command_success = None
    sequence_id = model.ros_clamps.send_ROS_VEL_GOTO_COMMAND(clamp_ids, position, velocity)

    # Wait for Clamp Controller to ACK
    while (True):
        if model.ros_clamps.sent_messages_ack[sequence_id] == True:
            logger_exe.info("Clamp Jaw Movement (%s) with %s to %smm Started" % (movement.movement_id, clamp_ids, position))
            break
        # # Check if the the reply is negative
        # if model.ros_clamps.sync_move_inaction == False:
        #     if model.ros_clamps.last_command_success == False:
        #         logger_exe.warn("Clamp Jaw Movement (%s) stopped because clamp ACK is not success." % (movement.movement_id))
        #         return False
        # Check if user pressed stop button in UI
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

        # Check if user pressed stop button in UI
        if model.run_status == RunStatus.STOPPED:
            logger_exe.warn(
                "UI stop button pressed before Robotic Clamp Jaw Movement (%s) is completed." % (movement.movement_id))
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
    future = send_and_wait_unless_cancel(model, rrc.GetJoints())
    if future.done:
        starting_robot_joints, starting_ext_axis = future.value
        logger_exe.info("Initial Joints: %s, %s" % (starting_robot_joints, starting_ext_axis))
    else:
        logger_exe.warn("UI stop button pressed before OperatorAddJogOffset (%s) GetJoints is completed." % (movement.movement_id))
        return False

    ############################################

    # Ask user to jog from TP and Stop Excution
    model.ros_robot.send(rrc.CustomInstruction('r_A067_TPPlot', ["Press STOP, JOG to align. Press PLAY when finished"], []))
    logger_exe.info("rrc.stop() issued")
    future = send_and_wait_unless_cancel(model, rrc.CustomInstruction('r_A067_Stop', [], []))
    if future.done:
        logger_exe.info("rrc.stop() returned")
    else:
        logger_exe.warn("UI stop button pressed before OperatorAddJogOffset (%s) rrc.Stop is completed." % (movement.movement_id))
        return False

    # Use UI Button to pause and confirm
    # button = guiref['exe']['confirm_button']
    # guiref['exe']['confirm_button_text'].set("Confirm after JOG")
    # button.config(state="normal", bg='orange')
    # model.operator_confirm = False

    # guiref['exe']['exe_status'].set("Paused")
    # from robot_clamp_controller.run import ui_update_run_status
    # while (True):
    #     if model.operator_confirm:
    #         button.config(state="disabled", bg='grey')
    #         ui_update_run_status(guiref, model)
    #         break
    #     if model.run_status == RunStatus.STOPPED:
    #         button.config(state="disabled", bg='grey')
    #         ui_update_run_status(guiref, model)
    #         return False

    ############################################

    # Upon restart, read the current robot frame.
    future = send_and_wait_unless_cancel(model, rrc.GetRobtarget())
    if future.done:
        new_frame, new_ext_axes = future.value
        logger_exe.info("Frame after Jog: %s" % (new_frame))
    else:
        logger_exe.warn("UI stop button pressed before OperatorAddJogOffset (%s) GetRobTarget is completed." % (movement.movement_id))
        return False

    future = send_and_wait_unless_cancel(model, rrc.GetJoints())
    if future.done:
        new_robot_joints, new_ext_axes = future.value
        logger_exe.info("Joints after Jog: %s, %s" % (new_robot_joints, new_ext_axes))
    else:
        logger_exe.warn("UI stop button pressed before OperatorAddJogOffset (%s) GetJoints is completed." % (movement.movement_id))
        return False
    ############################################

    # Compute offset and apply to external axis.
    offset = Vector.from_start_end(movement.original_frame.point, new_frame.point)
    logger_exe.info("Jogged Cartsian Offset = %s (amount = %4g mm)" % (offset, offset.length))

    # Compute Joint 6 offset
    end_state_robot_joints = to_degrees(movement.end_state['robot'].kinematic_config.revolute_values)
    joint_6_offset = new_robot_joints[5] - end_state_robot_joints[5]
    logger_exe.info("Jogged joint 6 offset = %s deg" % (joint_6_offset))

    # YZ axis of external axis has flipped direction
    guiref['offset']['Ext_X'].set("%.4g" % round(offset.x, 4))
    guiref['offset']['Ext_Y'].set("%.4g" % round(-1 * offset.y, 4))
    guiref['offset']['Ext_Z'].set("%.4g" % round(-1 * offset.z, 4))
    guiref['offset']['Rob_J1'].set("0")
    guiref['offset']['Rob_J2'].set("0")
    guiref['offset']['Rob_J3'].set("0")
    guiref['offset']['Rob_J4'].set("0")
    guiref['offset']['Rob_J5'].set("0")
    guiref['offset']['Rob_J6'].set("%.4g" % round(joint_6_offset, 4))

    # Movement to make sure joints have no offset
    # Ask user to jog from TP and Stop Excution
    model.ros_robot.send(rrc.CustomInstruction('r_A067_TPPlot', ["Offset registered: X %.3g Y %.3g Z %.3g." % (offset.x, offset.y, offset.z)], []))
    model.ros_robot.send(rrc.CustomInstruction('r_A067_TPPlot', ["Press PLAY to accept with a gantry move."], []))
    logger_exe.info("rrc.stop() issued")
    future = send_and_wait_unless_cancel(model, rrc.Stop())
    if not future.done:
        logger_exe.info("rrc.stop() returned")
        logger_exe.warn("UI stop button pressed before OperatorAddJogOffset (%s) rrc.Stop is completed." % (movement.movement_id))
        return False

    move_joint_instruction = robot_state_to_instruction(guiref, model, movement.end_state['robot'].kinematic_config, 30, rrc.Zone.FINE)
    future = send_and_wait_unless_cancel(model, move_joint_instruction)

    # Logging Gantry move is complete
    if future.done:
        model.ros_robot.send(rrc.CustomInstruction('r_A067_TPPlot', ["Gantry Move Complete"], []))
        logger_exe.info("OperatorAddJogOffset gantry move is complete")
    else:
        logger_exe.warn("UI stop button pressed before OperatorAddJogOffset (%s) TPPlot is completed." % (movement.movement_id))
        return False

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

#####################################################
# Execute functions that are not based on Movement
#####################################################

def execute_jog_robot_to_config(guiref, model, config: Configuration, message: str, q):
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
    ext_values = to_millimeters(config.prismatic_values)
    joint_values = to_degrees(config.revolute_values)

    # Apply Offsets
    ext_values = apply_ext_offsets(guiref, ext_values)
    joint_values = apply_joint_offsets(guiref, joint_values)

    if message != "":
        model.ros_robot.send(rrc.PrintText(message))

    instruction = rrc.MoveToJoints(joint_values, ext_values, 1000, rrc.Zone.FINE, feedback_level=rrc.FeedbackLevel.DONE)
    future = send_and_wait_unless_cancel(model, instruction)
    if future.done:
        logger_exe.info("execute_jog_robot_to_config complete")
        model.run_status = RunStatus.STOPPED
    else:
        logger_exe.warn("UI stop button pressed before MoveToJoints in JogRobotToState Movement is completed.")

    # Trigger update Status
    q.put(SimpleNamespace(type=BackgroundCommand.UI_UPDATE_STATUS))
    return future.done


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


def execute_acquire_docking_offset(guiref, model: RobotClampExecutionModel, movement: AcquireDockingOffset):
    """Performs AcquireDockingOffset by retriving the docking marker location.
    """
    if (model.ros_clamps is None) or not model.ros_clamps.is_connected:
        logger_exe.info(
            "AcquireDockingOffset cannot start because Clamp ROS is not connected")
        return False

    if not model.process.movement_has_end_robot_config(movement):
        logger_exe.warn("Error Attempt to execute execute_acquire_docking_offset but the movement end_state does not have robot config.")
        return False

    camera_stream_name = movement.tool_id
    logger_exe.info("Waiting for marker from camera %s" % (movement.tool_id))

    import roslibpy

    def markers_transformation_callback(message_string):

        # Print it to UI and keep track of one way latency.
        T = Transformation.from_data(json.loads(message_string['data']))
        model.ros_clamps.markers_transformation[camera_stream_name].append(T)
        logger_exe.debug("Received message_string = %s" % (message_string))

    # * Wait for marker transformation to come back
    max_iteration = 5
    for i in range(max_iteration):
        # * Reset list of observed transforamtion
        model.ros_clamps.markers_transformation[camera_stream_name] = []
        listener = roslibpy.Topic(model.ros_clamps, '/' + camera_stream_name, 'std_msgs/String')
        listener.subscribe(markers_transformation_callback)

        # * Store existing offset values
        prev_offset = []
        prev_offset.append(float(guiref['offset']['Ext_X'].get()))
        prev_offset.append(float(guiref['offset']['Ext_Y'].get()))
        prev_offset.append(float(guiref['offset']['Ext_Z'].get()))

        # * Acquire camera transformation and compute offset
        _last_check_index = 0
        while (True):
            # Check if camera frame have arrived or not

            num_markers = len(model.ros_clamps.markers_transformation[camera_stream_name])
            if num_markers > 5 and num_markers > _last_check_index:

                # * Double check the last two frames are agreeing with each other
                t_camera_from_observedmarker = model.ros_clamps.markers_transformation[camera_stream_name][-1]
                new_offset, correction_amount_XY, correction_amount_Z = compute_marker_correction(guiref, model, movement, t_camera_from_observedmarker)
                t_camera_from_observedmarker_2 = model.ros_clamps.markers_transformation[camera_stream_name][-2]
                new_offset_2, _, _ = compute_marker_correction(guiref, model, movement, t_camera_from_observedmarker_2)
                agreeable_threshold = 0.2
                difference = [abs(i-j) for i, j in zip(new_offset, new_offset_2)]

                # * If all difference agree, they can be
                if all([d < agreeable_threshold for d in difference]):
                    logger_exe.info("AcquireDockingOffset stream = %s attempt %s received t_camera_from_marker = %s" % (camera_stream_name, i, t_camera_from_observedmarker))
                    listener.unsubscribe()
                    break
                else:
                    logger_exe.info("AcquireDockingOffset last 2 frames correction larger than agreeable_threshold (%s) Difference = %s" % (agreeable_threshold, difference))
                _last_check_index = num_markers

            # Check if user pressed stop button in UI
            if model.run_status == RunStatus.STOPPED:
                logger_exe.warn(
                    "UI stop button pressed before AcquireDockingOffset (%s) is completed." % movement.movement_id)
                listener.unsubscribe()
                return False

        # * If the offsets are smaller than threshold (converged), break from multiple run loop and return
        convergence_XY = 1.0
        convergence_Z = 0.5
        if (correction_amount_XY < convergence_XY and correction_amount_Z < convergence_Z):
            logger_exe.info("Correction converged below threshold in %i move: XY = %1.2f (threshold = %1.2f), Z = %1.2f (threshold = %1.2f)" %
                            (i, correction_amount_XY, convergence_XY, correction_amount_Z, convergence_Z))
            return True

        # * Sanity check
        correction_max = 30
        if (correction_amount_XY > correction_max and correction_amount_Z > correction_max):
            logger_exe.warn("Correction larger than allowable threshold: XY = %1.2f (threshold = %1.2f), Z = %1.2f (threshold = %1.2f)" % (correction_amount_XY, correction_max, correction_amount_Z, correction_max))
            return False

        logger_exe.info("Correction amount (relative to current position) (in Flange Coordinates): XY = %1.2f , Z = %1.2f" % (correction_amount_XY, correction_amount_Z))

        # * Apply new offsets to existing offsets
        guiref['offset']['Ext_X'].set("%.4g" % round(prev_offset[0] + new_offset[0], 4))
        guiref['offset']['Ext_Y'].set("%.4g" % round(prev_offset[1] + new_offset[1], 4))
        guiref['offset']['Ext_Z'].set("%.4g" % round(prev_offset[2] + new_offset[2], 4))
        # * Move the robot to end config of movement with new offset
        config = model.process.get_movement_end_robot_config(movement)
        ext_values = apply_ext_offsets(guiref, to_millimeters(config.prismatic_values))
        joint_values = apply_joint_offsets(guiref, to_degrees(config.revolute_values))

        logger_exe.info("Moving robot to new offset value.")
        instruction = rrc.MoveToJoints(joint_values, ext_values, 500, rrc.Zone.FINE, feedback_level=rrc.FeedbackLevel.DONE)
        future = send_and_wait_unless_cancel(model, instruction)
        if future.done:
            logger_exe.info("Robot move to new offset value complete.")
        else:
            logger_exe.warn("UI stop button pressed before MoveToJoints in AcquireDockingOffset Movement is completed.")
            return False

    logger_exe.warn("AcquireDockingOffset exhausted maxIteration %s without convergence" % max_iteration)
    return False


def execute_some_delay(model: RobotClampExecutionModel, movement: Movement):
    for _ in range(10):
        time.sleep(0.3)
        if model.run_status == RunStatus.STOPPED:
            logger_exe.warn("UI stop button pressed before execute_some_delay (%s) is completed." % (movement.movement_id))
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


def compute_marker_correction(guiref, model: RobotClampExecutionModel, movement: AcquireDockingOffset, t_camera_from_observedmarker: Transformation, ):
    """Compute the gantry offset from the marker position.
    The movement must have a target_frame.

    Returns  (new_offset, correction_amount_XY, correction_amount_Z)
    - new_offset : three offset values that can be applied to gantry offset
    - correction_amount_XY : Correction amount in the Flange's coordinates
    - correction_amount_Z : Correction amount in the Flange's coordinates

    """

    # Retrive the selected movement target frame
    if not hasattr(movement, 'target_frame'):
        logger_model.warn("compute_visual_correction used on movement %s without target_frame" % (movement.movement_id))
        return False
    current_movement_target_frame = movement.target_frame
    t_world_from_currentflange = Transformation.from_frame(current_movement_target_frame)

    # * From CAD
    import clamp_controller
    import os.path as path
    camera_stream_name = movement.tool_id
    clamp_controller_path = path.dirname(path.dirname(path.dirname(clamp_controller.__file__)))
    json_path = path.join(clamp_controller_path, "calibrations", camera_stream_name + "_t_flange_from_marker.json")
    with open(json_path, 'r') as f:
        t_flange_from_marker = json.load(f, cls=DataDecoder)

    # * From Marker Calibration
    json_path = path.join(clamp_controller_path, "calibrations", camera_stream_name + "_t_camera_from_marker.json")
    with open(json_path, 'r') as f:
        t_camera_from_marker = json.load(f, cls=DataDecoder)

    # * Observation: Observed Marker in Camera Frame
    t_camera_from_observedmarker = t_camera_from_observedmarker

    # * Calcualtion: Camera in Flange Frame
    t_flange_from_camera = t_flange_from_marker * t_camera_from_marker.inverse()

    # * Calculation: New Flange Frame in current flange frame
    t_observedmarker_from_newflange = t_flange_from_marker.inverse()
    t_flange_from_newflange = t_flange_from_camera * t_camera_from_observedmarker * t_observedmarker_from_newflange  # type: Transformation
    v_flange_correction = t_flange_from_newflange.translation_vector
    v_world_flange_correction = v_flange_correction.transformed(t_world_from_currentflange)

    # * Convert correction vector to gantry offset values
    new_offset = []
    new_offset.append(v_world_flange_correction.x)
    new_offset.append(-1 * v_world_flange_correction.y)
    new_offset.append(-1 * v_world_flange_correction.z)

    # logger_exe.info("Marker correction t_flange_from_newflange = %s , v_world_flange_correction = %s" % (v_flange_correction, v_world_flange_correction))
    correction_amount_XY = Vector(v_flange_correction.x, v_flange_correction.y).length
    correction_amount_Z = v_flange_correction.z

    return (new_offset, correction_amount_XY, correction_amount_Z)


def compute_visual_correction(guiref, model: RobotClampExecutionModel, movement: RoboticMovement):
    """Compute the gantry offset from the visual offset in gui.
    The movement must have a target_frame"""
    align_X = guiref['offset']['Visual_X'].get()
    align_Y = guiref['offset']['Visual_Y'].get()
    align_Z = guiref['offset']['Visual_Z'].get()

    # Retrive the selected movement target frame
    if not hasattr(movement, 'target_frame'):
        logger_exe.warn("compute_visual_correction used on movement %s without target_frame" % (movement.movement_id))
        return False
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
    return True


#########################
# rrc Helper Functions
#########################

def send_and_wait_unless_cancel(model: RobotClampExecutionModel, instruction: ROSmsg) -> rrc.FutureResult:
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


def ensure_speed_ratio(model: RobotClampExecutionModel, robot:rrc.AbbClient, target_speed_ratio:float = 100):
    """Ensures the Speed Ratio of the robot matches the given `target_speed_ratio`.
    Otherwise, will write to TP to prompt user to change.
    User press PLAY after changing the ratio.

    Returns True if the user successfully changed the speed ratio to match.
    Returns False if
    """
    while (True):
        speed_ratio = robot.send_and_wait(GetSpeedRatio(), 2)
        print("speed_ratio = %s" % (speed_ratio))

        if abs(speed_ratio - target_speed_ratio) < 0.1:
            return True

        # Print Message to TP and ask for chaning the speed ratio.
        robot.send(rrc.CustomInstruction('r_A067_TPPlot', ['Change Speed Ratio to %i percent. Press PLAY to Continue.' % target_speed_ratio]))
        robot.send(rrc.Stop())
        robot.send_and_wait(rrc.WaitTime(0.1))
        # Allow UI to cancel
        if model.run_status == RunStatus.STOPPED:
            return False



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
    assert len(robot_config.prismatic_values) == 3
    assert len(robot_config.revolute_values) == 6

    # Ext Axis and Joint Values
    ext_values = to_millimeters(robot_config.prismatic_values)
    joint_values = to_degrees(robot_config.revolute_values)

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
