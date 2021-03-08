from enum import Enum, auto


class BackgroundCommand(Enum):
    UI_ROBOT_CONNECT = auto()
    UI_CLAMP_CONNECT = auto()

    UI_RUN = auto()
    UI_STEP = auto()
    UI_STOP = auto()
    UI_CONFIRM = auto()
    UI_UPDATE_STATUS = auto()
    UI_GOTO_END_FRAME = auto()
    UI_GOTO_START_STATE = auto()
    UI_GOTO_END_STATE = auto()
    UI_LOAD_EXT_MOVEMENT = auto()
    UI_OPEN_SETTING = auto()
    UI_SOFTMODE_ENABLE = auto()
    UI_SOFTMODE_DISABLE = auto()

    PRINT_ACTION_SUMMARY = auto()
    MODEL_LOAD_PROCESS = auto()
    EXE_CLAMPS_JAMMED = auto()
