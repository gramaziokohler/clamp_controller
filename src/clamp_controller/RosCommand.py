from typing import List, Tuple


class RequireMonitoring(object):

    def get_monitoring_tool_ids(self) -> List[str]:
        raise NotImplementedError


class ROS_COMMAND(object):
    """Base class for a command send via ROS to ClampController / RemoteToolController"""

    NOT_STARTED = 0
    RUNNING = 1
    SUCCEED = 2
    FAILED = 3

    def __init__(self):
        self.status = self.NOT_STARTED
        pass

    def to_data(self):
        return self.data

    @property
    def started(self):
        return self.status != self.NOT_STARTED

    @property
    def done(self):
        return self.status == self.SUCCEED or self.status == self.FAILED

    @property
    def success(self):
        return self.status == self.SUCCEED

    @classmethod
    def from_data(cls, data):
        command = cls()
        command.data = data
        return command

    @property
    def data(self):
        data = {
            'status': self.status
        }
        return data

    @data.setter
    def data(self, data):
        self.status = data['status']


class ROS_VEL_GOTO_COMMAND(ROS_COMMAND, RequireMonitoring):

    def __init__(self, clamps_pos_velo: List[Tuple[str, float, float]] = [], power_percentage=100, allowable_target_deviation=0):
        super().__init__()
        self.clamps_pos_velo = clamps_pos_velo
        self.power_percentage = power_percentage
        self.allowable_target_deviation = allowable_target_deviation

    @property
    def data(self):
        data = super(ROS_VEL_GOTO_COMMAND, self).data
        data['clamps_pos_velo'] = self.clamps_pos_velo,
        data['power_percentage'] = self.power_percentage,
        data['clamps_pos_velo'] = self.clamps_pos_velo,
        return data

    @data.setter
    def data(self, data):
        super(ROS_VEL_GOTO_COMMAND, type(self)).data.fset(self, data)
        self.clamps_pos_velo = data.get('clamps_pos_velo', [])
        self.power_percentage = data.get('power_percentage', 100)
        self.allowable_target_deviation = data.get('allowable_target_deviation', 0)

    def get_monitoring_tool_ids(self) -> List[str]:
        return [tool_id for tool_id, _, _ in self.clamps_pos_velo]


class ROS_STOP_COMMAND(ROS_COMMAND):
    def __init__(self, tools_id: List[str] = []):
        super().__init__()
        self.tools_id = tools_id

    @property
    def data(self):
        data = super(ROS_STOP_COMMAND, self).data
        data['tools_id'] = self.tools_id,
        return data

    @data.setter
    def data(self, data):
        super(ROS_STOP_COMMAND, type(self)).data.fset(self, data)
        self.tools_id = data.get('tools_id', [])


class ROS_STOP_ALL_COMMAND(ROS_COMMAND):
    pass


class ROS_SCREWDRIVER_GRIPPER_COMMAND(ROS_COMMAND, RequireMonitoring):
    def __init__(self, tool_id: str, extend: bool):
        super().__init__()
        self.tool_id = tool_id
        self.extend = extend

    @property
    def data(self):
        data = super(ROS_SCREWDRIVER_GRIPPER_COMMAND, self).data
        data['tools_id'] = self.tools_id,
        data['extend'] = self.extend,
        return data

    @data.setter
    def data(self, data):
        super(ROS_SCREWDRIVER_GRIPPER_COMMAND, type(self)).data.fset(self, data)
        self.tools_id = data.get('tools_id', [])
        self.extend = data.get('extend', [])

    def get_monitoring_tool_ids(self) -> List[str]:
        return [self.tool_id]


class ROS_REQUEST_STATUSUPDATE(ROS_COMMAND):
    """Request a status update reply"""
    pass


class ROS_COMMAND_STATUS(ROS_COMMAND):
    """Base class for a command send via ROS to ClampController / RemoteToolController"""

    def __init__(self, command: ROS_COMMAND, message_sequence_id: int):
        self.status = command.status
        self.message_sequence_id = message_sequence_id

    @property
    def data(self):
        data = {
            'status': self.status,
            'message_sequence_id': self.message_sequence_id,
        }
        return data

    @data.setter
    def data(self, data):
        super(ROS_COMMAND_STATUS, type(self)).data.fset(self, data)
        self.message_sequence_id = data.get('message_sequence_id', None)
