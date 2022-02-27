# This Python file uses the following encoding: utf-8
import json
import os

from ament_index_python import get_resource
from python_qt_binding import QtCore
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import QPushButton, QWidget
from shared.enums import ControlKeyEnum
from shared.inner_communication import innerCommunication
from shared.base_widget.base_widget import BaseWidget

from .elements.button import Button


class ControlPanelWidget(BaseWidget):
    def __init__(self, stack=None):
        super(ControlPanelWidget, self).__init__()

        self.stack = stack

        self.loadUI()

        innerCommunication.addRobotSignal.connect(self.initializeRobotsOptions)
        # innerCommunication.deleteRobotSignal.connect(self.onDeleteRobotSignal)

        self.setFocusPolicy(Qt.ClickFocus)
        self.setFocus()

        self.defineButtons()

        self.initializeRobotsOptions()

        self.comboBox.currentIndexChanged.connect(self.onChoosenRobotChange)
        currentData = self.comboBox.currentData()
        if currentData:
            self.dataFilePath = currentData['filePath']

            self.initializeSettings(self.dataFilePath)

    def loadUI(self):
        _, package_path = get_resource('packages', 'control_panel')
        ui_file = os.path.join(package_path, 'share', 'control_panel', 'resource', 'control_panel.ui')
        loadUi(ui_file, self)

    def test1(self):
        print('wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww aaaaaaaaaaaaa')


    def initializeSettings(self, filePath):
        dataFile = open(filePath)
        data = json.load(dataFile)
        dataFile.close()
        self.controlKeys = data['controlKeys']

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

        if key == self.controlKeys[ControlKeyEnum.FORWARD]:
            self.forwardButtonElement.pressedKeyState()
        elif event.key() == self.controlKeys[ControlKeyEnum.RIGHT]:
            self.rightButtonElement.pressedKeyState()
        elif event.key() == self.controlKeys[ControlKeyEnum.BACKWARD]:
            self.backwardButtonElement.pressedKeyState()
        elif event.key() == self.controlKeys[ControlKeyEnum.LEFT]:
            self.leftButtonElement.pressedKeyState()
        event.accept()

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return
        key = event.key()
        if key == self.controlKeys[ControlKeyEnum.FORWARD]:
            self.forwardButtonElement.releasedKeyState()
        elif event.key() == self.controlKeys[ControlKeyEnum.RIGHT]:
            self.rightButtonElement.releasedKeyState()
        elif event.key() == self.controlKeys[ControlKeyEnum.BACKWARD]:
            self.backwardButtonElement.releasedKeyState()
        elif event.key() == self.controlKeys[ControlKeyEnum.LEFT]:
            self.leftButtonElement.releasedKeyState()
        event.accept()

    def settingsClicked(self):
        self.stack.goToSettings(self.dataFilePath)

    def buttonClicked(self):
        pass

    def defineButtons(self):
        self.forwardButtonElement = Button(self.findChild(QPushButton, 'forwardButton'))
        self.rightButtonElement = Button(self.findChild(QPushButton, 'rightButton'))
        self.backwardButtonElement = Button(self.findChild(QPushButton, 'backwardButton'))
        self.leftButtonElement = Button(self.findChild(QPushButton, 'leftButton'))

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
