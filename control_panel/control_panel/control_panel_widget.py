# This Python file uses the following encoding: utf-8
import json
import os

from ament_index_python import get_resource
from python_qt_binding import QtCore
from python_qt_binding import loadUi
from shared.enums import ControlKeyEnum, MotorControlPositionEnum, motorControlPositionToDataKeyMap
from shared.base_widget.base_widget import BaseWidget

from .elements.button import Button
from minirys_msgs.msg import MotorCommand
from geometry_msgs.msg import Twist

from std_msgs.msg import Bool
from collections import namedtuple


class ControlPanelWidget(BaseWidget):
    def __init__(self, stack=None, node=None):
        super(ControlPanelWidget, self).__init__(stack)

        self.node = node

        self.setRobotOnScreen()

        self.defineButtons()

        self.balanceCheckBoxUI.stateChanged.connect(self.balanceStateChanged)

        self.initPressedKeys()

        self.messageTypeComboBoxUI.currentIndexChanged.connect(self.changeMessageFunction)
        self.messageFunction = self.setupTwistMessage
        self.publisher = node.create_publisher(Twist, self.namespace + '/cmd_vel', 10)

    def loadUI(self):
        _, packagePath = get_resource('packages', 'control_panel')
        uiFile = os.path.join(packagePath, 'share', 'control_panel', 'resource', 'control_panel.ui')
        loadUi(uiFile, self)

    def changeMessageFunction(self, event):
        if event == 0:
            self.messageFunction = self.setupTwistMessage
            self.publisher = self.node.create_publisher(Twist, self.namespace + '/cmd_vel', 10)

        elif event == 1:
            self.messageFunction = self.setupMotorCommandMessage
            self.publisher = self.node.create_publisher(MotorCommand, self.namespace + '/internal/motor_command', 10)

    def balanceStateChanged(self, state):
        msg = Bool()

        if state == 0:
            msg.data = False
            self.balancePublisher.publish(msg)
            return
        msg.data = True
        self.balancePublisher.publish(msg)

    def initPressedKeys(self):
        self.pressedKeys = {
            ControlKeyEnum.FORWARD: False,
            ControlKeyEnum.RIGHT: False,
            ControlKeyEnum.BACKWARD: False,
            ControlKeyEnum.LEFT: False
        }

    def initializeRobotSettings(self):
        self.controlKeys = self.data.get('controlKeys', {})
        self.dynamic = self.data.get('dynamic', {})
        self.dynamicTwist = self.data.get('dynamicTwist', {})

        self.balancePublisher = self.node.create_publisher(Bool, self.namespace + '/balance_mode', 10)

        self.changeMessageFunction(self.comboBox.currentIndex())

        for key in self.controlKeys:
            controlValue = self.controlKeys[key].upper()
            self.controlKeys[key] = QtCore.Qt.Key(ord(controlValue))

    def keyPressEvent(self, event):
        self.keyEvent(event, True)

    def keyReleaseEvent(self, event):
        self.keyEvent(event, False)

    def keyEvent(self, event, state):
        if event.isAutoRepeat():
            return
        button = None
        key = event.key()

        if key == self.controlKeys[ControlKeyEnum.FORWARD]:
            self.pressedKeys[ControlKeyEnum.FORWARD] = state
            button = self.forwardButtonElement
        elif key == self.controlKeys[ControlKeyEnum.RIGHT]:
            self.pressedKeys[ControlKeyEnum.RIGHT] = state
            button = self.rightButtonElement
        elif key == self.controlKeys[ControlKeyEnum.BACKWARD]:
            self.pressedKeys[ControlKeyEnum.BACKWARD] = state
            button = self.backwardButtonElement
        elif key == self.controlKeys[ControlKeyEnum.LEFT]:
            self.pressedKeys[ControlKeyEnum.LEFT] = state
            button = self.leftButtonElement

        if button:
            self.determineKeyedPressedState()
            if state:
                button.pressedKeyState()
            else:
                button.releasedKeyState()
        event.accept()

    def determineKeyedPressedState(self):
        keyState = self.getKeyState()

        msg = self.messageFunction(keyState)

        if msg is not None:
            self.publisher.publish(msg)

    def getKeyState(self):
        forward = self.pressedKeys[ControlKeyEnum.FORWARD]
        right = self.pressedKeys[ControlKeyEnum.RIGHT]
        backward = self.pressedKeys[ControlKeyEnum.BACKWARD]
        left = self.pressedKeys[ControlKeyEnum.LEFT]

        keyState = KeyState(forward, right, backward, left)
        return keyState

    def setupMotorCommandMessage(self, keyState):
        msg = MotorCommand()
        dataKey = KeyStateMap.get(keyState)

        if dataKey is not None:
            msg.speed_l = float(self.dynamic[dataKey]['leftEngine'])
            msg.speed_r = float(self.dynamic[dataKey]['rightEngine'])
        elif not (keyState.forward or keyState.right or keyState.backward or keyState.left):
            msg.speed_l = 0.0
            msg.speed_r = 0.0
        elif dataKey is None:
            return
        return msg

    def setupTwistMessage(self, keyState):
        msg = Twist()
        dataKey = KeyStateMap.get(keyState, None)

        if dataKey is not None:
            msg.linear.y = float(self.dynamicTwist[dataKey]['linear'])
            msg.angular.z = float(self.dynamicTwist[dataKey]['angle'])
        elif not (keyState.forward or keyState.right or keyState.backward or keyState.left):
            msg.linear.y = 0.0
            msg.angular.z = 0.0
        elif dataKey is None:
            return
        return msg

    def settingsClicked(self):
        self.stack.goToSettings(self.dataFilePath)

    def buttonClicked(self, isPressed, controlKeyEnum):
        self.pressedKeys[controlKeyEnum] = isPressed
        self.determineKeyedPressedState()

    def defineButtons(self):
        self.settingsButton.clicked.connect(self.settingsClicked)

        self.forwardButtonElement = Button(self.forwardButton)
        self.rightButtonElement = Button(self.rightButton)
        self.backwardButtonElement = Button(self.backwardButton)
        self.leftButtonElement = Button(self.leftButton)

        self.forwardButton.pressed.connect(lambda: self.buttonClicked(True, ControlKeyEnum.FORWARD))
        self.rightButton.pressed.connect(lambda: self.buttonClicked(True, ControlKeyEnum.RIGHT))
        self.backwardButton.pressed.connect(lambda: self.buttonClicked(True, ControlKeyEnum.BACKWARD))
        self.leftButton.pressed.connect(lambda: self.buttonClicked(True, ControlKeyEnum.LEFT))

        self.forwardButton.released.connect(lambda: self.buttonClicked(False, ControlKeyEnum.FORWARD))
        self.rightButton.released.connect(lambda: self.buttonClicked(False, ControlKeyEnum.RIGHT))
        self.backwardButton.released.connect(lambda: self.buttonClicked(False, ControlKeyEnum.BACKWARD))
        self.leftButton.released.connect(lambda: self.buttonClicked(False, ControlKeyEnum.LEFT))

    def resizeEvent(self, event):
        self.setIconSize()

    def setIconSize(self):
        width = int(self.backwardButton.size().width() * 0.9)
        height = int(self.backwardButton.size().height() * 0.9)

        self.forwardButtonElement.resizeIcon(width, height)
        self.leftButtonElement.resizeIcon(width, height)
        self.rightButtonElement.resizeIcon(width, height)
        self.backwardButtonElement.resizeIcon(width, height)


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
