from enum import Enum

class BackgroundCommand(Enum):
    UI_ROS_CONNECT = 0
    UI_RUN = 1
    UI_STEP = 2
    UI_STOP = 3
    UI_CONFIRM = 4
    UI_UPDATE_STATUS = 5
    MODEL_LOAD_PROCESS = 10
    EXE_CLAMPS_JAMMED = 11