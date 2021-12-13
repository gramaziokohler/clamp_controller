import datetime
import logging
import time
from typing import Dict, List, Optional, Tuple

import serial.tools.list_ports
from serial import Serial

from clamp_controller.ClampModel import ClampModel
from serial_radio_transport_driver.Message import Message
from serial_radio_transport_driver.ReliableMessenger import ReliableMessenger
from serial_radio_transport_driver.SerialTransport import SerialRadioTransport


def current_milli_time(): return int(round(time.time() * 1000))


class SerialCommander(object):

    def __init__(self):
        self.clamps: Dict[str, ClampModel] = {} # Key of the dictionary is the Receiver Address, typical in range of (str)"1" to (str)"7"
        self.serial_port = None
        self.status_update_interval_high_ms: int = 150  # ms
        self.status_update_interval_low_ms: int = 2000  # ms
        self.serial_default_retry: int = 3
        self.serial_default_timeout: int = 100
        self.logger = logging.getLogger("app.cmd")
        self.status_update_high_freq: bool = False  # Flag that indicate update status interval to be in high frequency (when clamps are in motion)
        self.sync_move_inaction = False             # Flag to indicate sync move in action and monitor if any clamp stopped.
        self.sync_move_clamp_pos_velo_list = None  # type: List[Tuple[ClampModel, float, float]]
        self.last_command_success = True
        pass

    @property
    def is_connected(self) -> bool:
        return ((self.serial_port is not None) and (self.serial_port.isOpen()))

    @ property
    def status_update_interval_ms(self) -> int:
        if self.status_update_high_freq:
            return self.status_update_interval_high_ms
        else:
            return self.status_update_interval_low_ms

    def connect_with_CLI(self) -> bool:
        """" Launch a CLI based interface to allow user to select a COM PORT.
        Connect to that COM port
        return true if success
        """
        # Check connected COM ports
        ports = list(serial.tools.list_ports.comports())
        if len(ports) == 0:
            print("No COM Ports available. Terminating")
            return False

        # If there is more than one list available, list the Serial Ports to user
        if len(ports) > 1:
            print("The following COM Ports are connected. Please select the USB Radio Dongle:")
            for i, p in enumerate(ports):
                print("<%s> %s" % (i, p))
            selected_index = int(input("- Which port <?>"))
            selected_port = ports[selected_index][0]

        # If there is only one COM Port select that
        else:
            selected_port = ports[0][0]

        return self.connect(selected_port)

    def connect(self, port_name: str):
        """ Connect to a specified COM Port by Name
        return True if success
        """
        # Check if existing port is connected, if so, disconned first:
        if (self.serial_port is not None):
            if (self.serial_port.isOpen()):
                self.logger.info("Closing existing port: %s" % (self.serial_port.port))
                self.serial_port.close()

        # Connect to Serial Port
        self.logger.info("Connecting to %s ..." % (port_name))
        try:
            serial_port = Serial(port_name, 115200, timeout=1)
            serial_port.reset_input_buffer()
        except:
            self.logger.error("Connection failed. Check if other termianals are already connected to %s" % (port_name))
            return False

        # Need to wait for a while for ARduino to Reset
        time.sleep(3)
        self.serial_port = serial_port
        # Prepare SerialTransport
        self.transport = SerialRadioTransport(serial_port, address='0', eom_char='\n')
        self.logger.info("Connected to %s." % (serial_port.port))
        return True

    def add_clamp(self, clamp: ClampModel) -> None:
        """ Add a ClampModel instance to be managed by this commander
        """
        clamp.last_comm_time = None
        #clamp.last_comm_lqi = None
        #clamp.last_comm_rssi = None
        clamp.last_comm_latency = 0
        self.clamps[clamp.receiver_address] = clamp

    def get_clamp_by_process_tool_id(self, process_tool_id:str):
        for clamp in self.clamps.values():
            if clamp.process_tool_id == process_tool_id:
                return clamp
        raise ValueError("get_clamp_by_process_tool_id() failed, process_tool_id %s not found." % process_tool_id)

    def check_status_update_freq(self):
        """ Start or stop high frequency updates.
        Based on if any clamps are running."""

        if not self.status_update_high_freq:
            # * Logic to turn high freq ON
            if any([clamp.isMotorRunning for clamp in self.clamps.values()]):
                self.status_update_high_freq = True
                self.logger.info("Clamps started moving. Status Update Freq = High")
            pass
        else:
            # * Logic to turn high freq OFF
            if not any([clamp.isMotorRunning for clamp in self.clamps.values()]):
                self.status_update_high_freq = False
                self.logger.info("All clamps have stopped. Status Update Freq = Low")

    def update_clamps_status(self, clamps: List[ClampModel], retry: int = 0, time_out_millis: int = 40) -> List[ClampModel]:
        """ Update the status of the given clamps.
        Returns a list of clamps that have been successfully updated

        If some clamps are running, update frequency will increase automatically.
        """
        results = []
        for clamp in clamps:
            success = self.update_clamp_status(clamp, retry=retry, time_out_millis=time_out_millis)
            if success:
                results.append(clamp)
        self.check_status_update_freq()
        return results

    def update_all_clamps_status(self, retry: int = 0, time_out_millis: int = 40) -> List[ClampModel]:
        """ Update the status of all clamps.
        Returns a list of clamps that have been successfully updated

        If some clamps are running, update frequency will increase automatically.
        """
        results = self.update_clamps_status(self.clamps.values(), retry=retry, time_out_millis=time_out_millis)
        return results

    def update_active_clamps_status(self, retry: int = 0, time_out_millis: int = 40) -> List[ClampModel]:
        """ Update the status of all active clamps:

        - If self.sync_move_inaction == true, then active_clamps == clamps in the sync_move_list
        - Else  active_clamps == clamps where clamp.isMotorRunning == true

        Returns a list of clamps that have been successfully updated.

        If some clamps are running, update frequency will increase automatically.
        """
        results = []

        if self.sync_move_inaction:
            sync_move_list = [clamp for (clamp, pos, velo) in self.sync_move_clamp_pos_velo_list]
            active_clamps = [clamp for clamp in self.clamps.values() if clamp in sync_move_list]
        else:
            active_clamps = [clamp for clamp in self.clamps.values() if clamp.isMotorRunning]

        results = self.update_clamps_status(active_clamps, retry=retry, time_out_millis=time_out_millis)
        return results

    def update_clamp_status(self, clamp: ClampModel, retry: int = 0, time_out_millis: int = 40) -> bool:
        """ Update the status of one clamp
        """
        response = self.message_clamp(clamp, "_", retry=retry, time_out_millis=time_out_millis, update_clamp_status=False)
        if response is not None:
            # Update clamp status
            update_result = clamp.update_status(response)
            self.logger.debug("Status Update Success: %s result = %s" % (clamp, response))

            return update_result
        else:
            self.logger.debug("Status Update Fail: %s No-Response" % (clamp))
            return False

    def message_clamp(self, clamp: ClampModel, message: str, retry: int = -1, time_out_millis: int = -1, update_clamp_status: bool = True) -> str:
        # Get default communication values
        if (retry == -1):
            retry = self.serial_default_retry
        if (time_out_millis == -1):
            time_out_millis = self.serial_default_timeout

        self.transport.reset_input_buffer()
        transmit_start_millis = current_milli_time()
        # Multiple sending attempts
        for tries in range(retry + 1):
            try:
                # Sometimes send_message() can fail. if that is the case, we skip the rest of this try loop
                self.transport.send_message(Message(clamp.receiver_address, '0', message))
            except:
                continue
            # Allow time to receive status response
            receive_timeout = current_milli_time() + time_out_millis
            while (current_milli_time() < receive_timeout):
                received_message = self.transport.receive_message()
                if received_message is None:
                    continue
                if (received_message.sender_address != clamp.receiver_address):
                    continue
                # Save communication statistics in ClampModel
                clamp.last_comm_time = time.time()
                clamp.last_comm_latency = current_milli_time() - transmit_start_millis
                # Update the status of the clamp
                if update_clamp_status:
                    update_success = clamp.update_status(received_message.body)
                return (received_message.body)
        # In case there are no response after all attempts
        return None

    def sync_clamps_move(self, clamp_pos_velo_list: List[Tuple[ClampModel, float, float]]) -> bool:
        """Commands a list of clamps to perform a syncronous move
        """
        # This higher level function does not perform value min max check.

        # clamp_pos_velo_list is a list of clamps to syncrously move.
        # It is a list of Tuples. Containing
        #   - clamp: ClampModel
        #   - jaw_position_mm:float
        #   - velocity_mm_sec:float

        # Set Velocity / The values can be different for each clamp
        successes = []
        for clamp, jaw_position_mm, velocity_mm_sec in clamp_pos_velo_list:
            success = self.set_clamp_velocity(clamp, velocity_mm_sec, retry=3)
            successes.append(success)
        if not all(successes):
            self.logger.warning("Sync Clamp Move Not Successful at velocity setting step. Successes: %s" % successes)
            return False

        # Send Goto Command
        processed_clamps = []
        for clamp, jaw_position_mm, velocity_mm_sec in clamp_pos_velo_list:
            success = self.send_clamp_to_jaw_position(clamp, jaw_position_mm, retry=2)
            processed_clamps.append(clamp)
            if not success:
                self.logger.warning("Sync Clamp Move Not Successful, %s no response." % processed_clamps[-1])
                self.stop_clamps(processed_clamps)
                return False

        # Keep the status of this sync move for later monitoring
        self.sync_move_inaction = True
        self.sync_move_clamp_pos_velo_list = clamp_pos_velo_list
        # If nothing went wrong, this is a success
        return True

    # Return True if messages are successfully sent and acked
    def send_clamp_to_jaw_position(self, clamp: ClampModel, jaw_position_mm: float, retry: int = 2) -> bool:
        # Check position min max
        if (jaw_position_mm < clamp.SoftLimitMin_mm):
            # raise ValueError("Target Position (%s) < Limit (%s)" % (jaw_position_mm, clamp.SoftLimitMin_mm))
            self.logger.error("Target Position (%s) < Limit (%s)" % (jaw_position_mm, clamp.SoftLimitMin_mm))
            return False
        if (jaw_position_mm > clamp.SoftLimitMax_mm):
            # raise ValueError("Target Position (%s) > Limit (%s)" % (jaw_position_mm, clamp.SoftLimitMax_mm))
            self.logger.error("Target Position (%s) > Limit (%s)" % (jaw_position_mm, clamp.SoftLimitMax_mm))
            return False

        # Convert velocity and target into step units
        motor_position = int(clamp.to_motor_position(jaw_position_mm))

        # Send Target to Go
        message = "g" + str(motor_position)
        response = self.message_clamp(clamp, message, retry=retry)
        success = response is not None

        # Returns True if the send is successful
        if success:
            clamp._last_set_position = jaw_position_mm
            if not self.status_update_high_freq:
                self.logger.info("send_clamp_to_jaw_position() Issued. Status Update Freq = High")
                self.status_update_high_freq = True
        self.logger.info("send_clamp_to_jaw_position(%s,%s), message = %s, success = %s" % (clamp, jaw_position_mm, message, success))
        return success

    def home_clamps(self, clamps: List[ClampModel]) -> List[bool]:
        """Command multiple clamps to home.
        """
        successes = []
        for clamp in clamps:
            response = self.message_clamp(clamp, "h")
            successes.append(response is not None)
        if any(successes):
            if not self.status_update_high_freq:
                self.logger.info("home_clamps() Issued. Status Update Freq = High")
                self.status_update_high_freq = True
        return successes

    def stop_clamps(self, clamps: List[ClampModel]) -> List[bool]:
        """Stop multiple clamps
        """
        successes = []
        for clamp in clamps:
            response = self.message_clamp(clamp, "s")
            successes.append(response is not None)
        return successes

    def stop_all_clamps(self) -> List[bool]:
        """Command all clamps managed by this controller to stop immediately
        """
        successes = self.stop_clamps(self.clamps.values())
        if all(successes):
            self.logger.info("stop_clamps() issued. All clamps are stopped. Status Update Freq = Low")
            self.status_update_high_freq = False
        return successes

    def set_clamp_velocity(self, clamp: ClampModel, velocity_mm_sec: float, retry: int = 3) -> bool:
        """ Set the velocity setting of a clamp
        """
        # Check velocity min max
        this_clamp_VelocityMin_mm_sec = 0.01  # TO Do. Integrate this into clampModel
        this_clamp_VelocityMax_mm_sec = 10.0  # TO Do. Integrate this into clampModel
        if (velocity_mm_sec < this_clamp_VelocityMin_mm_sec):
            #raise ValueError("Target Velocity (%s) < Limit (%s)" % (velocity_mm_sec, this_clamp_VelocityMin_mm_sec))
            self.logger.error("Target Velocity (%s) < Limit (%s)" % (velocity_mm_sec, this_clamp_VelocityMin_mm_sec))
            return False
        if (velocity_mm_sec > this_clamp_VelocityMax_mm_sec):
            #raise ValueError("Target Velocity (%s) > Limit (%s)" % (velocity_mm_sec, this_clamp_VelocityMax_mm_sec))
            self.logger.error("Target Velocity (%s) > Limit (%s)" % (velocity_mm_sec, this_clamp_VelocityMax_mm_sec))
            return False

        # Convert units
        velocity_step_sec = int(velocity_mm_sec * clamp.StepPerMM)

        # Send message to clamp
        message = "v" + str(velocity_step_sec)
        response = self.message_clamp(clamp, message, retry=retry)
        success = response is not None
        # Record this last sent value
        if success:
            clamp._last_set_velocity = velocity_mm_sec
        self.logger.info("set_clamp_velocity(%s,%s), message = %s, success = %s" % (clamp, velocity_mm_sec, message, success))
        return success

    def set_clamps_velocity(self, clamps: List[ClampModel], velocity_mm_sec: float, retry: int = 3) -> List[bool]:
        """Set the velocity setting of multiple clamps
        Return a list of boolean values whether the command is successfully ACKed.
        """
        successes = []
        for clamp in clamps:
            success = self.set_clamp_velocity(clamp, velocity_mm_sec, retry=retry)
            successes.append(success)
        return successes

    def set_clamp_power(self, clamp: ClampModel, power_pct: float) -> bool:
        """ Set the power setting of a clamp
        """
        # Check power min max
        this_clamp_power_Min_pct = 1  # TO Do. Integrate this into clampModel
        this_clamp_power_Max_pct = 100  # TO Do. Integrate this into clampModel
        if (power_pct < this_clamp_power_Min_pct):
            #raise ValueError("Target Power (%s) < Limit (%s)" % (power_pct, this_clamp_power_Min_pct))
            self.logger.error("Target Power (%s) < Limit (%s)" % (power_pct, this_clamp_power_Min_pct))
            return False
        if (power_pct > this_clamp_power_Max_pct):
            #raise ValueError("Target Power (%s) > Limit (%s)" % (power_pct, this_clamp_power_Max_pct))
            self.logger.error("Target Power (%s) > Limit (%s)" % (power_pct, this_clamp_power_Max_pct))
            return False

        # Send message to clamp
        message = "p%0.1f" % (power_pct)
        response = self.message_clamp(clamp, message)
        success = response is not None
        # Record this last sent value
        if success:
            clamp._last_set_power = power_pct
        self.logger.info("set_clamp_power(%s,%s), message = %s, success = %s" % (clamp, power_pct, message, success))
        return success

    def set_clamps_power(self, clamps: List[ClampModel], power_pct: float) -> List[bool]:
        """Set the power setting of multiple clamps
        power_pct can range from 0.0 to 100.0
        Return a list of boolean values whether the command is successfully ACKed.
        """
        successes = []
        for clamp in clamps:
            success = self.set_clamp_power(clamp, power_pct)
            successes.append(success)
        return successes
