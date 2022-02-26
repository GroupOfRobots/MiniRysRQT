from enum import Enum


class ControlKeyEnum(str, Enum):
    FORWARD = 'forward'
    RIGHT = 'right'
    BACKWARD = 'backward'
    LEFT = 'left'
    STABLE = 'stable' # position is preserved if stable key is pressed

