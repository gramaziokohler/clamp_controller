# RemoteClampFunctionCall.py provides simple to use functions to send clamp commands via ROS
# The receiving Clamp Controller should have be started.

# Typical usage:
#   hostip = '192.168.43.141'
#   clamps_connection = RemoteClampFunctionCall(hostip)
# Command to send clamp to target (non-blocking)
#   clamps_connection.send_ROS_VEL_GOTO_COMMAND(100.0, 1.0)
# Command to send clamp to target (blocking)
#   success = clamps_connection.send_ROS_VEL_GOTO_COMMAND_wait(100.0, 1.0, 1000)
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
def current_milli_time(): return int(round(time.time() * 1000))


class RemoteClampFunctionCall(Ros):

    def __init__(self, host_ip, status_change_callback=None):
        if not compas.IPY:
            from twisted.internet import reactor
            reactor.timeout = lambda: 0.00001

        Ros.__init__(self, host=host_ip, port=9090)

        # Communication control
        self.sequence_id = -1

        self.sent_messages_ack = {}
        self.trip_times = []
        self.external_status_change_callback = status_change_callback
        self.last_command_success = None
        self.markers_transformation = {}

        # * Keeps the status update send back fom the Clamp Controller
        self.last_status_time = 0       # Keep track of the (local) time stamp of last status update
        self.clamps_status_count = 0
        self.clamps_status = {}         # Raw Clamp Status sent back from the Clamp Controller
        self.sync_move_inaction = None  # Flag whether the last sent sync_move command is successful

        def clamp_response_callback(message_string):
            receive_time = current_milli_time()
            # Retrive the sent message and compare time
            feedback_message = json.loads(message_string['data'])
            sequence_id = feedback_message['sequence_id']
            org_message = self.sent_messages_ack[sequence_id]
            org_message['received'] = True
            send_time = int(org_message['timestamp'])
            bounce_time = int(feedback_message['timestamp'])

            # Print it to UI and keep track of one way latency.
            rtt = receive_time - send_time
            t1t = bounce_time - send_time
            t2t = receive_time - bounce_time
            # print ("Received; %s" % feedback_message)
            print("Received Message %s, RoundTripTime = %s (%s + %s)" % (sequence_id, rtt, t1t, t2t))
            self.trip_times.append(rtt)

            if 'ack' in feedback_message:
                ack_success = int(feedback_message['ack'])
                self.sent_messages_ack[sequence_id] = ack_success
                print('Received Message: ack received = %s' % ack_success)

        # Setup talker to send message
        self.talker = roslibpy.Topic(self, '/clamp_command', 'std_msgs/String')
        self.talker.advertise()

        # Setup listener to listen for feedback
        self.listener = roslibpy.Topic(self, '/clamp_response', 'std_msgs/String')
        self.listener.subscribe(clamp_response_callback)

        # Clamp_status are status messages published by the clamp controller.
        # There is no need to repond to these messages. But perhaps need updating the model upstream.
        #
        def status_callback(message):
            data = json.loads(message['data'])
            # Relay message to callback
            if self.external_status_change_callback is not None:
                self.external_status_change_callback(data)

            self.clamps_status_count += 1
            self.clamps_status = data['status']
            self.sync_move_inaction = data['sync_move_inaction']
            self.last_command_success = data['last_command_success']
            self.last_status_time = time.time()

        # Setup listener topic.
        self.listener = roslibpy.Topic(self, '/clamp_status', 'std_msgs/String')
        self.listener.subscribe(status_callback)

        # Send an initial message Sequence id will be -1
        self.send_ros_command("ROS_NEW_SENDER_INITIALIZE", "")

    def send_ros_command(self, instruction_type, instruction_body):
        """ Sends a ROS command to /clamp_command channel and returns the sequence_id of sent message
        instruction_type: str
        instruction_body; str
        """
        # Create message to send
        message = {}
        message['sequence_id'] = self.sequence_id
        message['timestamp'] = current_milli_time()
        message['instruction_type'] = instruction_type
        message['instruction_body'] = instruction_body
        self.talker.publish(roslibpy.Message({'data': json.dumps(message)}))
        # Keep track of sent message
        message['received'] = False  # Keep track of ACK.
        self.sent_messages_ack[self.sequence_id] = message
        self.sequence_id += 1
        print("Sent Message to clamp_command:", message)
        return self.sequence_id - 1

    # Return true if sending is successful. False if timeout
    def send_ROS_VEL_GOTO_COMMAND_wait(self, clamps_id, position, velocity, timeout_ms=1000):
        """clamps_id: str, position: float, velocity: float, timeout_ms: int = 1000) -> bool:
        """
        start_time = current_milli_time()
        sequence_id = self.send_ROS_VEL_GOTO_COMMAND(clamps_id, position, velocity)
        while (current_milli_time() - start_time < timeout_ms):
            if self.sent_messages_ack[sequence_id] == True:
                return True

        return False

    # Returns the sequence_id of sent message
    def send_ROS_GRIPPER_OPEN_COMMAND(self, screwdriver_id: str):
        """Move grippers on a screwdriver to open / retracted position."""
        return self.send_ros_command("ROS_GRIPPER_OPEN_COMMAND", screwdriver_id)

    def send_ROS_GRIPPER_CLOSE_COMMAND(self, screwdriver_id: str):
        """Move grippers on a screwdriver to close / extended position."""
        return self.send_ros_command("ROS_GRIPPER_CLOSE_COMMAND", screwdriver_id)

    # Returns the sequence_id of sent message
    def send_ROS_VEL_GOTO_COMMAND(self, clamps_id: List[str], position: float, velocity: float):
        """Move multiple clamps to the same position using same velocity"""
        instructions = []
        for clamp_id in clamps_id:
            instructions.append((clamp_id, position, velocity))
        return self.send_ros_command("ROS_VEL_GOTO_COMMAND", instructions)

    # Returns the sequence_id of sent message
    def send_ROS_STOP_COMMAND(self, clamps_id: List[str]):
        "Stop multiple clamps"
        instructions = []
        for clamp_id in clamps_id:
            instructions.append((clamp_id))
        return self.send_ros_command("ROS_STOP_COMMAND", instructions)

    def send_ROS_STOP_ALL_COMMAND(self):
        "Stop multiple clamps"
        return self.send_ros_command("ROS_STOP_ALL_COMMAND","")

    def terminate(self):
        self.talker.unadvertise()
        Ros.terminate(self)


# CLI Loop
if __name__ == "__main__":

    hostip = '192.168.0.117'
    clamps_connection = RemoteClampFunctionCall(hostip)
    clamps_connection.run()
    # Command to send clamp to target (non-blocking)
    # clamps_connection.send_ROS_VEL_GOTO_COMMAND(100.0, 1.0)

    # Command to send clamp to target (blocking)
    # success = clamps_connection.send_ROS_VEL_GOTO_COMMAND_wait(100.0, 1.0, 1000)

    while clamps_connection.is_connected:
        i = input("Type a position (95-220) , velocity  (0.1 - 3.0) (to Send a test Message topic=/clamp_command , x to quit.\n")
        # Function to Stop UI
        if i == 'x':
            break
        if i == 's':
            # Stop
            clamps_connection.send_ROS_STOP_COMMAND(['3'])
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
            success = clamps_connection.send_ROS_VEL_GOTO_COMMAND_wait(['3'], position, velocity)
            if success:
                print("send_ROS_VEL_GOTO_COMMAND_wait() Message Success (Clamp Ack)")
            else:
                print("send_ROS_VEL_GOTO_COMMAND_wait() Message Fail (NACK)")
        # place message in dict to allow later retrivel for time comparision.

    clamps_connection.terminate()
