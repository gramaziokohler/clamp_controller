# RosClampCommandListener allows a ClampController to accept commands from ROS
#
# This is intended as a singleton class that provides the functions to
# subscribe and listens to a ROS topic that publish commands
#
# New commands are passed on to a callback function that has to be provided at init.

import datetime
import json
import time

from typing import List, Dict, Tuple, Callable

import roslibpy
from roslibpy import Ros
from clamp_controller.RosCommand import *
from clamp_controller.RemoteClampFunctionCall import RosMessage

def current_milli_time(): return int(round(time.time() * 1000))


class RosClampCommandListener(Ros):

    def __init__(self, host, command_callback: Callable[[ROS_COMMAND, RosMessage], None], is_secure=False):


        from twisted.internet import reactor
        reactor.timeout = lambda : 0.00001
        Ros.__init__(self, host, 9090, is_secure)

        self.received_command_message_pairs = {} # type: Dict[ROS_COMMAND, RosMessage]
        self.command_callback = command_callback # type: Callable[[ROS_COMMAND, RosMessage]]

        # Setup replier topic.
        self.message_ack_replier = roslibpy.Topic(self, '/clamp_message_ack', 'std_msgs/String')
        self.message_ack_replier.advertise()

        def clamp_command_receive_callback(received_roslibpy_message):
            # * Reconstruct RosMessage
            message = RosMessage.from_received_roslibpy_message(received_roslibpy_message)

            # * Reconstruct Command
            if message.command is None:
                print("RosMessage received with no command. : %s" % message)

            # Ack the message with a empty message but same id
            ack_message = {}
            ack_message = RosMessage(message.sequence_id)
            self.message_ack_replier.publish(ack_message.to_roslibpy_message())

            # Store the received command message pair
            self.received_command_message_pairs[message.command] = message
            # Relay message to callback
            self.command_callback(message.command, message)

        # Setup listener topic.
        self.listener = roslibpy.Topic(self, '/clamp_message', 'std_msgs/String')
        self.listener.subscribe(clamp_command_receive_callback)

        self.status_publisher = roslibpy.Topic(self, '/clamp_command_status', 'std_msgs/String')
        self.status_publisher.advertise()


    def send_command_status(self, command: ROS_COMMAND):
        """ Sends a ROS command to /clamp_command channel and returns the sequence_id of sent message
        instruction_type: str
        instruction_body; str
        """
        # Create message to send
        reply_dict={}
        orig_message = self.received_command_message_pairs[command]
        # reply_command = ROS_COMMAND_STATUS(command, orig_message.sequence_id)
        reply_command = ROS_COMMAND()
        reply_command.status =command.status
        self.status_publisher.publish(RosMessage.from_command(orig_message.sequence_id, reply_command).to_roslibpy_message())

# Directly calling this script creates a listener that will print out messages.
# It will also show the one way trip time.


if __name__ == "__main__":

    hostip = '192.168.0.120'

    trip_times = []

    def command_callback(command: ROS_COMMAND, message:RosMessage):
        print(command.data)
        trip_time = current_milli_time() - message.send_time
        trip_times.append(trip_time)
        print("Trip Time = %s" % trip_time)

    client = RosClampCommandListener(hostip, command_callback)
    client.run()

    while (client.is_connected):
        i = input("Press x to quit and see average trip time. \n")
        if i == 'x':
            break

    print("Total %s messages received, trip time = %s" % (len(trip_times), sum(trip_times) / len(trip_times)))

    client.terminate()
