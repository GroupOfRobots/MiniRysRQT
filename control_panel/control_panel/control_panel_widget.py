# This Python file uses the following encoding: utf-8
import json
import os

from ament_index_python import get_resource
from python_qt_binding import QtCore
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import QPushButton, QWidget
from shared.enums import ControlKeyEnum
from shared.base_widget.base_widget import BaseWidget

from .elements.button import Button

import rclpy
from rclpy.node import Node
from minirys_msgs.msg import MotorCommand

from std_msgs.msg import Float32, String


class ControlPanelWidget(BaseWidget):
    def __init__(self, stack=None,node=None):
        super(ControlPanelWidget, self).__init__()
        BaseWidget.__init__(self, stack)

        self.loadUI()

        self.defineButtons()

        self.initializeRobotsOptions()

        self.comboBox.currentIndexChanged.connect(self.onChoosenRobotChange)
        currentData = self.comboBox.currentData()
        if currentData:
            self.dataFilePath = currentData['filePath']

            self.initializeSettings(self.dataFilePath)

        self.node = node
        self.publisher = self.node.create_publisher(MotorCommand, '/internal/motor_command', 10)
        self.publisher2 = self.node.create_publisher(Float32, '/internal/fan_output', 10)

    def loadUI(self):
        _, packagePath = get_resource('packages', 'control_panel')
        uiFile = os.path.join(packagePath, 'share', 'control_panel', 'resource', 'control_panel.ui')
        loadUi(uiFile, self)

    def initializeSettings(self, filePath):
        dataFile = open(filePath)
        data = json.load(dataFile)
        dataFile.close()
        self.controlKeys = data['controlKeys']
        self.dynamic = data['dynamic']

        for key in self.controlKeys:
            controlValue = self.controlKeys[key].upper()
            self.controlKeys[key] = QtCore.Qt.Key(ord(controlValue))

    def onChoosenRobotChange(self, event):
        data = self.comboBox.currentData()
        if data:
            self.setRobotOnScreen(data)

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return
        key = QtCore.Qt.Key(event.key())
        msg = MotorCommand()

        a = Float32()
        a.data = 0.3
        self.publisher2.publish(a)

        if key == self.controlKeys[ControlKeyEnum.FORWARD]:
            self.forwardButtonElement.pressedKeyState()
            # print(self.dynamic)
            msg.speed_l = float(self.dynamic['forward']['leftEngine'])
            msg.speed_r = float(self.dynamic['forward']['rightEngine'])
            print(msg)
            self.publisher.publish(msg)
        elif event.key() == self.controlKeys[ControlKeyEnum.RIGHT]:
            msg.speed_l = 20.0
            msg.speed_r = 20.0
            self.publisher.publish(msg)

            self.rightButtonElement.pressedKeyState()
        elif event.key() == self.controlKeys[ControlKeyEnum.BACKWARD]:
            msg.speed_l = -20.0
            msg.speed_r = 20.0
            self.publisher.publish(msg)

            self.backwardButtonElement.pressedKeyState()
        elif event.key() == self.controlKeys[ControlKeyEnum.LEFT]:
            msg.speed_l = -20.0
            msg.speed_r = -20.0
            self.publisher.publish(msg)
            self.leftButtonElement.pressedKeyState()
        event.accept()

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return
        key = event.key()
        msg = MotorCommand()
        msg.speed_l = 0.0
        msg.speed_r = 0.0

        # a= Float32(0)
        a = Float32()
        a.data = 0.0

        self.publisher2.publish(a)

        if key == self.controlKeys[ControlKeyEnum.FORWARD]:

            self.publisher.publish(msg)
            self.forwardButtonElement.releasedKeyState()
        elif event.key() == self.controlKeys[ControlKeyEnum.RIGHT]:
            self.publisher.publish(msg)

            self.rightButtonElement.releasedKeyState()
        elif event.key() == self.controlKeys[ControlKeyEnum.BACKWARD]:
            self.publisher.publish(msg)

            self.backwardButtonElement.releasedKeyState()
        elif event.key() == self.controlKeys[ControlKeyEnum.LEFT]:
            self.publisher.publish(msg)

            self.leftButtonElement.releasedKeyState()
        event.accept()

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
        width = self.backwardButton.size().width() * 0.9
        height = self.backwardButton.size().height() * 0.9
        self.forwardButtonElement.resizeIcon(width, height)
        self.leftButtonElement.resizeIcon(width, height)
        self.rightButtonElement.resizeIcon(width, height)
        self.backwardButtonElement.resizeIcon(width, height)
