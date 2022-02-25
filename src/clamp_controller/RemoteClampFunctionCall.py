# RemoteClampFunctionCall.py provides simple to use functions to send clamp commands via ROS
# The receiving Clamp Controller should have be started.

# Typical usage:
#   hostip = '192.168.43.141'
#   clamps_connection = RemoteClampFunctionCall(hostip)
# Command to send clamp to target (non-blocking)
#   clamps_connection.send(ROS_VEL_GOTO_COMMAND)
# Command to send clamp to target (blocking)
#   success = clamps_connection.send_and_wait(ROS_VEL_GOTO_COMMAND, 1000)
# Command to stop clamps (non-blocking)
#   clamps_connection.send_ROS_STOP_COMMAND(['1','2'])

# Directly running this file creates a CLI for sending movement commands.

import compas
from typing import List, Dict, Tuple
from roslibpy import Ros
import time
import datetime
import json
import roslibpy
import logging

from clamp_controller.RosCommand import *

logger_ctr = logging.getLogger("app.remote_clamp_call")

def current_milli_time(): return int(round(time.time() * 1000))


class RosMessage(object):

    def __init__(self, sequence_id: int) -> None:
        self.sequence_id = sequence_id
        self.send_time = None  # type: int # ms
        self.command = None  # type: ROS_COMMAND
        self.reply_receive_time = None  # type: int # ms
        # self.reply_type = None # type: str
        # self.reply_data = None # type: Dict

    @classmethod
    def from_command(cls, sequence_id: int, command: ROS_COMMAND):
        message = cls(sequence_id)
        message.command = command
        return message

    @property
    def command_type(self):
        if self.command is not None:
            return self.command.__class__.__name__
        else:
            return None

    @property
    def command_data(self):
        if self.command is not None:
            return self.command.data
        else:
            return None

    def to_sendable_dict(self) -> Dict:
        data = {
            "sequence_id": self.sequence_id,
            "send_time": self.send_time,
            "command_type": self.command_type,
            "command_data": self.command_data,
        }
        return data

    def to_sendable_str(self) -> str:
        return json.dumps(self.to_sendable_dict())

    def to_roslibpy_message(self) -> roslibpy.Message:
        return roslibpy.Message({'data': self.to_sendable_str()})

    @classmethod
    def from_received_dict(cls, dict: Dict):
        sequence_id = dict['sequence_id']
        message = cls(sequence_id)
        message.send_time = dict['send_time']

        # Reconstruct the ROS_COMMAND
        if dict['command_type'] == "ROS_VEL_GOTO_COMMAND":
            message.command = ROS_VEL_GOTO_COMMAND.from_data(dict['command_data'])
        if dict['command_type'] == "ROS_STOP_COMMAND":
            message.command = ROS_STOP_COMMAND.from_data(dict['command_data'])
        if dict['command_type'] == "ROS_STOP_ALL_COMMAND":
            message.command = ROS_STOP_ALL_COMMAND.from_data(dict['command_data'])
        if dict['command_type'] == "ROS_SCREWDRIVER_GRIPPER_COMMAND":
            message.command = ROS_SCREWDRIVER_GRIPPER_COMMAND.from_data(dict['command_data'])
        if dict['command_type'] == "ROS_REQUEST_STATUSUPDATE":
            message.command = ROS_REQUEST_STATUSUPDATE.from_data(dict['command_data'])
        if dict['command_type'] == "ROS_COMMAND_STATUS":
            message.command = ROS_COMMAND_STATUS.from_data(dict['command_data'])
        if dict['command_type'] == "ROS_COMMAND":
            message.command = ROS_COMMAND.from_data(dict['command_data'])

        return message

    @classmethod
    def from_received_str(cls, received_str: str):
        dict = json.loads(received_str)
        return cls.from_received_dict(dict)

    @classmethod
    def from_received_roslibpy_message(cls, roslibpy_message: str):
        received_str = roslibpy_message['data']
        return cls.from_received_str(received_str)

    @property
    def acked(self):
        """Indicates that the receiver have received and acknowledged the message"""
        return self.reply_receive_time is not None


class RemoteClampFunctionCall(Ros):

    def __init__(self, host_ip, status_change_callback=None):
        if not compas.IPY:
            from twisted.internet import reactor
            reactor.timeout = lambda: 0.00001

        Ros.__init__(self, host=host_ip, port=9090)

        # Communication control
        self.sequence_id = 0

        self.sent_messages = {}  # type: Dict[int, RosMessage]
        self.external_status_change_callback = status_change_callback
        self.last_command_success = None
        self.markers_transformation = {}

        # * Keeps the status update send back fom the Clamp Controller
        self.last_received_message_time = 0
        self.last_status_time = 0       # Keep track of the (local) time stamp of last status update
        self.clamps_status_count = 0
        self.clamps_status = {}         # Raw Clamp Status sent back from the Clamp Controller
        self.sync_move_inaction = None  # Flag whether the last sent sync_move command is successful

        def clamp_message_ack_callback(received_roslibpy_message):
            """Handles the recepion of message ACK from the ToolController"""
            # Retrive the sent message and compare time
            if 'sequence_id' not in received_roslibpy_message['data']:
                return
            # Retrive original sent message
            message = RosMessage.from_received_roslibpy_message(received_roslibpy_message)
            sequence_id = message.sequence_id
            org_message = self.sent_messages[sequence_id]

            # Mark reply receive time
            org_message.reply_receive_time = current_milli_time()
            self.last_received_message_time = current_milli_time()

            # Print it to UI and keep track of one way latency.
            rtt = org_message.reply_receive_time - org_message.send_time
            print('Message %i ACK received. RoundTripTime = %s ' % (sequence_id, rtt))

        # Setup talker to send message
        self.talker = roslibpy.Topic(self, '/clamp_message', 'std_msgs/String')
        self.talker.advertise()

        # Setup listener to listen for feedback
        self.listener = roslibpy.Topic(self, '/clamp_message_ack', 'std_msgs/String')
        self.listener.subscribe(clamp_message_ack_callback)

        # Clamp_status are status messages published by the clamp controller.
        # There is no need to repond to these messages. But perhaps need updating the model upstream.
        #
        def status_callback(received_roslibpy_message):
            """Handles the command status message from the ToolController"""
            # Retrive original sent message
            message = RosMessage.from_received_roslibpy_message(received_roslibpy_message)
            sequence_id = message.sequence_id

            # Retrive original sent message and change status
            org_message = self.sent_messages[sequence_id]
            org_message.command.status = message.command.status

            # Mark receive time
            self.last_received_message_time = current_milli_time()
            print('Message %i StatusUpdate Received. status = %s ' % (sequence_id, message.command.status))

        # Setup listener topic.
        self.status_listener = roslibpy.Topic(self, '/clamp_command_status', 'std_msgs/String')
        self.status_listener.subscribe(status_callback)

        # Send an initial message Sequence id will be -1
        self.send(ROS_REQUEST_STATUSUPDATE())

    def send(self, command: ROS_COMMAND) -> RosMessage:
        """ Sends a ROS command

        Internally we create a ROS_Message

        Returns the sent RosMessage.
        """
        message = RosMessage.from_command(self.sequence_id, command)
        message.send_time = current_milli_time()

        # * We send the entire RosMessage object
        self.talker.publish(message.to_roslibpy_message())

        # * Keep track of the sent message
        self.sent_messages[message.sequence_id] = message
        print("Message %i Sent. sendable_dict=%s" %(message.sequence_id, message.to_sendable_dict()))
        self.sequence_id += 1  # Increment message counter
        return message

    def send_and_wait(self, command: ROS_COMMAND, timeout_ms: float = 1000) -> bool:
        message = self.send(command)
        while (current_milli_time() - message.send_time < timeout_ms):
            if message.reply_receive_time is not None:
                return True
        return False

    @property
    def last_received_message_age_ms(self):
        return current_milli_time() - self.last_received_message_time

    def terminate(self):
        self.talker.unadvertise()
        Ros.terminate(self)


# CLI Loop
if __name__ == "__main__":

    hostip = '192.168.0.33'
    clamps_connection = RemoteClampFunctionCall(hostip)
    clamps_connection.run()
    # Command to send clamp to target (non-blocking)
    # clamps_connection.send(ROS_VEL_GOTO_COMMAND)

    # Command to send clamp to target (blocking)
    # success = clamps_connection.send_and_wait(ROS_VEL_GOTO_COMMAND, 1000)
    clamp_id = 's1'
    power_percentage = None
    while clamps_connection.is_connected:
        i = input("Type a position (95-220) , velocity  (0.1 - 3.0) (to Send a test Message topic=/clamp_command , x to quit.\n")
        # Function to Stop UI
        if i == 'x':
            break
        if i == 's':
            # Stop
            clamps_connection.send(ROS_STOP_COMMAND([clamp_id]))
            continue
        else:
            # Goto Pos,Vel
            try:
                _pos, _vel = i.split(',')
                position = float(_pos)
                velocity = float(_vel)
                if (position < 95.0 or position > 220.0):
                    raise ValueError
            except:
                print("Bad Input, position range (95-220). Try again:\n")
                continue
            command = ROS_VEL_GOTO_COMMAND([(clamp_id, position, velocity)], power_percentage, 2)
            success = clamps_connection.send_and_wait(command)
            if success:
                print("send_and_wait() Message Success (Clamp Ack)")
            else:
                print("send_and_wait() Message Fail (NACK)")

            # while (True):
            #     if command()
        # place message in dict to allow later retrivel for time comparision.

    clamps_connection.terminate()
