from enum import Enum


class ControlKeyEnum(str, Enum):
    FORWARD = 'forward'
    RIGHT = 'right'
    BACKWARD = 'backward'
    LEFT = 'left'
    STABLE = 'stable'  # position is preserved if stable key is pressed


class MotorControlPositionInTableEnum(int, Enum):
    FORWARD = 0
    FORWARD_LEFT = 1
    FORWARD_RIGHT = 2
    RIGHT = 3
    BACKWARD = 4  # position is preserved if stable key is pressed
    BACKWARD_LEFT = 5
    BACKWARD_RIGHT = 6
    LEFT = 7


motorControlPositionToDataKeyMap = {
    MotorControlPositionInTableEnum.FORWARD: 'forward',
    MotorControlPositionInTableEnum.FORWARD_LEFT: 'forwardLeft',
    MotorControlPositionInTableEnum.FORWARD_RIGHT: 'forwardRight',
    MotorControlPositionInTableEnum.RIGHT: 'right',
    MotorControlPositionInTableEnum.BACKWARD: 'backward',
    MotorControlPositionInTableEnum.BACKWARD_LEFT: 'backwardLeft',
    MotorControlPositionInTableEnum.BACKWARD_RIGHT: 'backwardRight',
    MotorControlPositionInTableEnum.LEFT: 'left'
}
