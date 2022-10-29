from enum import Enum


class ControlKeyEnum(str, Enum):
    FORWARD = 'forward'
    RIGHT = 'right'
    BACKWARD = 'backward'
    LEFT = 'left'
    STABLE = 'stable'  # position is preserved if stable key is pressed


class MotorControlPositionEnum(int, Enum):
    FORWARD = 0
    FORWARD_LEFT = 1
    FORWARD_RIGHT = 2
    RIGHT = 3
    BACKWARD = 4  # position is preserved if stable key is pressed
    BACKWARD_LEFT = 5
    BACKWARD_RIGHT = 6
    LEFT = 7


motorControlPositionToDataKeyMap = {
    MotorControlPositionEnum.FORWARD: 'forward',
    MotorControlPositionEnum.FORWARD_LEFT: 'forwardLeft',
    MotorControlPositionEnum.FORWARD_RIGHT: 'forwardRight',
    MotorControlPositionEnum.RIGHT: 'right',
    MotorControlPositionEnum.BACKWARD: 'backward',
    MotorControlPositionEnum.BACKWARD_LEFT: 'backwardLeft',
    MotorControlPositionEnum.BACKWARD_RIGHT: 'backwardRight',
    MotorControlPositionEnum.LEFT: 'left'
}
