from collections import namedtuple

from shared.enums import ControlKeyEnum, MotorControlPositionEnum, motorControlPositionToDataKeyMap


def getKeyState(pressedKeys):
    forward = pressedKeys[ControlKeyEnum.FORWARD]
    right = pressedKeys[ControlKeyEnum.RIGHT]
    backward = pressedKeys[ControlKeyEnum.BACKWARD]
    left = pressedKeys[ControlKeyEnum.LEFT]

    keyState = KeyState(forward, right, backward, left)
    return keyState


KeyState = namedtuple('KeyState', ["forward", "right", "backward", "left"])

KeyStateMap = {
    KeyState(True, False, False, False): motorControlPositionToDataKeyMap[MotorControlPositionEnum.FORWARD],
    KeyState(True, True, False, False): motorControlPositionToDataKeyMap[MotorControlPositionEnum.FORWARD_RIGHT],
    KeyState(True, False, False, True): motorControlPositionToDataKeyMap[MotorControlPositionEnum.FORWARD_LEFT],
    KeyState(False, True, False, False): motorControlPositionToDataKeyMap[MotorControlPositionEnum.RIGHT],
    KeyState(False, False, True, False): motorControlPositionToDataKeyMap[MotorControlPositionEnum.BACKWARD],
    KeyState(False, True, True, False): motorControlPositionToDataKeyMap[MotorControlPositionEnum.BACKWARD_RIGHT],
    KeyState(False, False, True, True): motorControlPositionToDataKeyMap[MotorControlPositionEnum.BACKWARD_LEFT],
    KeyState(False, False, False, True): motorControlPositionToDataKeyMap[MotorControlPositionEnum.LEFT]
}
