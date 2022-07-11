# This Python file uses the following encoding: utf-8
import json
import os

from ament_index_python import get_resource
from python_qt_binding import QtCore
from python_qt_binding import loadUi
from shared.enums import ControlKeyEnum
from shared.base_widget.base_widget import BaseWidget

from .elements.button import Button
from minirys_msgs.msg import MotorCommand

class ControlPanelWidget(BaseWidget):
    def __init__(self, stack=None, node=None):
        super(ControlPanelWidget, self).__init__()
        BaseWidget.__init__(self, stack)

        self.loadUI()
        self.initializeRobotsOptions()
        self.setRobotOnScreen()

        # self.initializeRobotSettings()

        self.defineButtons()

        self.comboBox.currentIndexChanged.connect(self.setRobotOnScreen)

        self.pressedKeys = {
            ControlKeyEnum.FORWARD: False,
            ControlKeyEnum.RIGHT: False,
            ControlKeyEnum.BACKWARD: False,
            ControlKeyEnum.LEFT: False
        }
        self.publisher = node.create_publisher(MotorCommand, '/internal/motor_command', 10)

    def loadUI(self):
        _, packagePath = get_resource('packages', 'control_panel')
        uiFile = os.path.join(packagePath, 'share', 'control_panel', 'resource', 'control_panel.ui')
        loadUi(uiFile, self)

    def initializeRobotSettings(self):
        self.controlKeys = self.data.get('controlKeys', {})
        self.dynamic = self.data.get('dynamic',{})

        for key in self.controlKeys:
            controlValue = self.controlKeys[key].upper()
            self.controlKeys[key] = QtCore.Qt.Key(ord(controlValue))

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return
        key = QtCore.Qt.Key(event.key())

        if key == self.controlKeys[ControlKeyEnum.FORWARD]:
            self.forwardButtonElement.pressedKeyState()
            self.pressedKeys[ControlKeyEnum.FORWARD] = True
        elif event.key() == self.controlKeys[ControlKeyEnum.RIGHT]:
            self.pressedKeys[ControlKeyEnum.RIGHT] = True
            self.rightButtonElement.pressedKeyState()
        elif event.key() == self.controlKeys[ControlKeyEnum.BACKWARD]:
            self.pressedKeys[ControlKeyEnum.BACKWARD] = True
            self.backwardButtonElement.pressedKeyState()
        elif event.key() == self.controlKeys[ControlKeyEnum.LEFT]:
            self.pressedKeys[ControlKeyEnum.LEFT] = True
            self.leftButtonElement.pressedKeyState()

        self.determineKeyedPressedState()

        event.accept()

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return
        key = event.key()

        if key == self.controlKeys[ControlKeyEnum.FORWARD]:
            self.pressedKeys[ControlKeyEnum.FORWARD] = False
            self.forwardButtonElement.releasedKeyState()
        elif event.key() == self.controlKeys[ControlKeyEnum.RIGHT]:
            self.pressedKeys[ControlKeyEnum.RIGHT] = False
            self.rightButtonElement.releasedKeyState()
        elif event.key() == self.controlKeys[ControlKeyEnum.BACKWARD]:
            self.pressedKeys[ControlKeyEnum.BACKWARD] = False
            self.backwardButtonElement.releasedKeyState()
        elif event.key() == self.controlKeys[ControlKeyEnum.LEFT]:
            self.pressedKeys[ControlKeyEnum.LEFT] = False
            self.leftButtonElement.releasedKeyState()

        self.determineKeyedPressedState()

        event.accept()

    def determineKeyedPressedState(self):
        forward = self.pressedKeys[ControlKeyEnum.FORWARD]
        right = self.pressedKeys[ControlKeyEnum.RIGHT]
        backward = self.pressedKeys[ControlKeyEnum.BACKWARD]
        left = self.pressedKeys[ControlKeyEnum.LEFT]
        msg = MotorCommand()

        if forward and right and not (backward or left):
            msg.speed_l = float(self.dynamic['forwardRight']['leftEngine'])
            msg.speed_r = float(self.dynamic['forwardRight']['rightEngine'])
        elif forward and left and not (backward or left):
            msg.speed_l = float(self.dynamic['forwardLeft']['leftEngine'])
            msg.speed_r = float(self.dynamic['forwardLeft']['rightEngine'])
        elif forward and not (right or backward or left):
            msg.speed_l = float(self.dynamic['forward']['leftEngine'])
            msg.speed_r = float(self.dynamic['forward']['rightEngine'])

        elif right and not (forward or backward or left):
            msg.speed_l = float(self.dynamic['right']['leftEngine'])
            msg.speed_r = float(self.dynamic['right']['rightEngine'])

        if backward and right and not (forward or left):
            msg.speed_l = float(self.dynamic['backwardRight']['leftEngine'])
            msg.speed_r = float(self.dynamic['backwardRight']['rightEngine'])
        elif backward and left and not (forward or left):
            msg.speed_l = float(self.dynamic['backwardLeft']['leftEngine'])
            msg.speed_r = float(self.dynamic['backwardLeft']['rightEngine'])
        elif backward and not (right or forward or left):
            msg.speed_l = float(self.dynamic['backward']['leftEngine'])
            msg.speed_r = float(self.dynamic['backward']['rightEngine'])

        elif left and not (forward or backward or right):
            msg.speed_l = float(self.dynamic['left']['leftEngine'])
            msg.speed_r = float(self.dynamic['left']['rightEngine'])

        elif not (forward or right or backward or left):
            msg.speed_l = 0.0
            msg.speed_r = 0.0

        self.publisher.publish(msg)

    def settingsClicked(self):
        self.stack.goToSettings(self.dataFilePath)

    def buttonClicked(self):
        pass

    def defineButtons(self):
        self.forwardButtonElement = Button(self.forwardButton)
        self.rightButtonElement = Button(self.rightButton)
        self.backwardButtonElement = Button(self.backwardButton)
        self.leftButtonElement = Button(self.leftButton)

        self.settingsButton.clicked.connect(self.settingsClicked)
        self.forwardButton.clicked.connect(self.buttonClicked)

    def resizeEvent(self, event):
        self.setIconSize()

    def setIconSize(self):
        width = int(self.backwardButton.size().width() * 0.9)
        height = int(self.backwardButton.size().height() * 0.9)
        self.forwardButtonElement.resizeIcon(width, height)
        self.leftButtonElement.resizeIcon(width, height)
        self.rightButtonElement.resizeIcon(width, height)
        self.backwardButtonElement.resizeIcon(width, height)
