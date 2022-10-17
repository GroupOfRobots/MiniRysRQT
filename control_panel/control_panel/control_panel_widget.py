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
from geometry_msgs.msg import Twist

from std_msgs.msg import Bool


class ControlPanelWidget(BaseWidget):
    def __init__(self, stack=None, node=None):
        super(ControlPanelWidget, self).__init__(stack)

        self.loadUI()
        self.initializeRobotsOptions()
        self.setRobotOnScreen()

        self.defineButtons()

        self.comboBox.currentIndexChanged.connect(self.setRobotOnScreen)
        self.balanceCheckBoxUI.stateChanged.connect(self.balanceStateChanged)

        self.initPressesKeys()

        self.publisherTwist = node.create_publisher(Twist, self.namespace + '/internal/cmd_vel', 10)
        self.publisher = node.create_publisher(MotorCommand, self.namespace + '/internal/motor_command', 10)
        self.balancePublisher = node.create_publisher(Bool,  self.namespace +'/balance_mode', 10)

    def loadUI(self):
        _, packagePath = get_resource('packages', 'control_panel')
        uiFile = os.path.join(packagePath, 'share', 'control_panel', 'resource', 'control_panel.ui')
        loadUi(uiFile, self)

    def balanceStateChanged(self, state):
        msg = Bool()

        if state == 0:
            msg.data = False
            self.balancePublisher.publish(msg)
            return
        msg.data = True
        self.balancePublisher.publish(msg)

    def initPressesKeys(self):
        self.pressedKeys = {
            ControlKeyEnum.FORWARD: False,
            ControlKeyEnum.RIGHT: False,
            ControlKeyEnum.BACKWARD: False,
            ControlKeyEnum.LEFT: False
        }

    def initializeRobotSettings(self):
        self.controlKeys = self.data.get('controlKeys', {})
        self.dynamic = self.data.get('dynamic', {})

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

        self.determineKeyedPressedState1()

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
        elif forward and left and not (backward or right):
            msg.speed_l = float(self.dynamic['forwardLeft']['leftEngine'])
            msg.speed_r = float(self.dynamic['forwardLeft']['rightEngine'])
        elif forward and not (right or backward or left):
            msg.speed_l = float(self.dynamic['forward']['leftEngine'])
            msg.speed_r = float(self.dynamic['forward']['rightEngine'])

        elif right and not (forward or backward or left):
            msg.speed_l = float(self.dynamic['right']['leftEngine'])
            msg.speed_r = float(self.dynamic['right']['rightEngine'])

        elif backward and right and not (forward or left):
            msg.speed_l = float(self.dynamic['backwardRight']['leftEngine'])
            msg.speed_r = float(self.dynamic['backwardRight']['rightEngine'])
        elif backward and left and not (forward or right):
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

    def determineKeyedPressedState1(self):
        forward = self.pressedKeys[ControlKeyEnum.FORWARD]
        right = self.pressedKeys[ControlKeyEnum.RIGHT]
        backward = self.pressedKeys[ControlKeyEnum.BACKWARD]
        left = self.pressedKeys[ControlKeyEnum.LEFT]
        msg = Twist()

        if forward and right and not (backward or left):
            msg.linear.y = float(self.dynamic['forwardRight']['linear'])
            msg.angular.z = float(self.dynamic['forwardRight']['angle'])
        elif forward and left and not (backward or right):
            msg.linear.y = float(self.dynamic['forwardLeft']['linear'])
            msg.angular.z = float(self.dynamic['forwardLeft']['angle'])
        elif forward and not (right or backward or left):
            msg.linear.y = float(self.dynamic['forward']['linear'])
            msg.angular.z = float(self.dynamic['forward']['angle'])

        elif right and not (forward or backward or left):
            msg.linear.y = float(self.dynamic['right']['linear'])
            msg.angular.z = float(self.dynamic['right']['angle'])

        elif backward and right and not (forward or left):
            msg.linear.y = float(self.dynamic['backwardRight']['linear'])
            msg.angular.z = float(self.dynamic['backwardRight']['angle'])
        elif backward and left and not (forward or right):
            msg.linear.y = float(self.dynamic['backwardLeft']['linear'])
            msg.angular.z = float(self.dynamic['backwardLeft']['angle'])
        elif backward and not (right or forward or left):
            msg.linear.y = float(self.dynamic['backward']['linear'])
            msg.angular.z = float(self.dynamic['backward']['angle'])

        elif left and not (forward or backward or right):
            msg.linear.y = float(self.dynamic['left']['linear'])
            msg.angular.z = float(self.dynamic['left']['angle'])

        elif not (forward or right or backward or left):
            msg.linear.y = 0.0
            msg.angular.z = 0.0

        self.publisherTwist.publish(msg)

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
