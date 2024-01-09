import threading

from python_qt_binding import QtCore
from shared.enums import ControlKeyEnum


class KeyPressService:
    def __init__(self, controlKeys):
        self.controlKeys = controlKeys
        self.pressedKeys = []
        self.xMove = 0
        self.yMove = 0
        self.keyPressedThread = threading.Thread()

    def onKeyPressed(self, event):
        key = event.key()
        if key == self.controlKeys[ControlKeyEnum.FORWARD]:
            self.pressedKeys.append(ControlKeyEnum.FORWARD)
            self.yMove = -1
        elif key == self.controlKeys[ControlKeyEnum.RIGHT]:
            self.pressedKeys.append(ControlKeyEnum.RIGHT)
            self.xMove = 1
        elif key == self.controlKeys[ControlKeyEnum.BACKWARD]:
            self.pressedKeys.append(ControlKeyEnum.BACKWARD)
            self.yMove = 1
        elif key == self.controlKeys[ControlKeyEnum.LEFT]:
            self.pressedKeys.append(ControlKeyEnum.LEFT)
            self.xMove = -1
        elif key == self.controlKeys[ControlKeyEnum.STABLE]:
            self.pressedKeys.append(ControlKeyEnum.STABLE)
        else:
            event.accept()
            return False

        return True

    def onKeyReleaseEvent(self, event):
        key = event.key()
        try:
            if key == self.controlKeys[ControlKeyEnum.FORWARD]:
                self.pressedKeys.remove(ControlKeyEnum.FORWARD)
                if ControlKeyEnum.BACKWARD in self.pressedKeys:
                    self.yMove = 1
                else:
                    self.yMove = 0
            elif key == self.controlKeys[ControlKeyEnum.RIGHT]:
                self.pressedKeys.remove(ControlKeyEnum.RIGHT)
                if ControlKeyEnum.LEFT in self.pressedKeys:
                    self.xMove = -1
                else:
                    self.xMove = 0
            elif key == self.controlKeys[ControlKeyEnum.BACKWARD]:
                self.pressedKeys.remove(ControlKeyEnum.BACKWARD)
                if ControlKeyEnum.FORWARD in self.pressedKeys:
                    self.yMove = -1
                else:
                    self.yMove = 0
            elif key == self.controlKeys[ControlKeyEnum.LEFT]:
                self.pressedKeys.remove(ControlKeyEnum.LEFT)
                if ControlKeyEnum.RIGHT in self.pressedKeys:
                    self.xMove = 1
                else:
                    self.xMove = 0
            elif key == self.controlKeys[ControlKeyEnum.STABLE]:
                self.pressedKeys.remove(ControlKeyEnum.STABLE)
            else:
                event.accept()
                return False
        except Exception as exception:
            return False
        return True
