# This Python file uses the following encoding: utf-8
import os

import rclpy

from std_msgs.msg import String

from python_qt_binding import QtCore
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import QPushButton, QWidget, QComboBox
from ament_index_python import get_resource
from python_qt_binding import loadUi

from .elements.button import Button


import logging
from watchdog.observers import Observer
from watchdog.events import LoggingEventHandler

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import json

class ControlPanelWidget(QWidget):
    def __init__(self, node, plugin=None):
        super(ControlPanelWidget, self).__init__()

        self.node = node
        self.controlPanelStack =plugin

        _, package_path = get_resource('packages', 'control_panel')
        ui_file = os.path.join(package_path, 'share', 'control_panel', 'resource', 'control_panel.ui')
        loadUi(ui_file, self)

        logging.basicConfig(level=logging.INFO,
                            format='%(asctime)s - %(message)s',
                            datefmt='%Y-%m-%d %H:%M:%S')

        _, package_path1 = get_resource('packages', 'setup_panel')
        self.dataFilePath =  os.path.join(package_path1, 'share', 'setup_panel', 'data', 'robots')

        event_handler = LoggingEventHandler()
        observer = Observer()
        observer.schedule(event_handler, self.dataFilePath, recursive=True)
        observer.start()


        self.setFocusPolicy(Qt.ClickFocus)
        self.setFocus()

        self.defineButtons()
        self.initializeRobotOptions()

        print(self.comboBox.currentText())
        self.comboBox.currentIndexChanged.connect(self.onChoosenRobotChange)
        robotName=self.comboBox.currentText()
        filePath = self.nameToFileDictionary[robotName]
        self.initializeSettings(filePath)

    def initializeRobotOptions(self):
        robotNamesList = []
        self.nameToFileDictionary = {}
        self.fileToNameDictionary = {}

        for index,fileName in enumerate(os.listdir(self.dataFilePath)):
            filePath = self.dataFilePath+'/'+fileName
            dataFile = open(filePath)
            data = json.load(dataFile)
            dataFile.close()
            robotName = data['robotName']
            robotNamesList.append(robotName)

            self.fileToNameDictionary[filePath] = robotName
            self.nameToFileDictionary[robotName] = filePath

        self.comboBox.addItems(robotNamesList)

    def initializeSettings(self, filePath):
        dataFile = open(filePath)
        data = json.load(dataFile)
        dataFile.close()
        self.controlKeys = data['controlKeys']

        # print(self.controlKeys)


    def onChoosenRobotChange(self,event):
        print(event)

    def keyPressEvent(self, event):
        print(event)
        print(event.text())
        # print(event.matches('W'))

        print(QtCore.Qt.Key_W)
        print((ord('W')))
        print((ord("w")))
        print(QtCore.Qt.Key(10))
        print(QtCore.Qt.Key(ord('w')))
        print(QtCore.Qt.Key_W== QtCore.Qt.Key(ord('w')))
        print(QtCore.Qt.Key_W== QtCore.Qt.Key(ord('W')))

        if event.isAutoRepeat():
            return
        key = event.key()
        if key == QtCore.Qt.Key_W:
            self.forwardButtonElement.pressedKeyState()
        elif event.key() == QtCore.Qt.Key_D:
            self.rightButtonElement.pressedKeyState()
        elif event.key() == QtCore.Qt.Key_S:
            self.backwardButtonElement.pressedKeyState()
        elif event.key() == QtCore.Qt.Key_A:
            self.leftButtonElement.pressedKeyState()
        event.accept()

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return
        key = event.key()
        if key == QtCore.Qt.Key_W:
            self.forwardButtonElement.releasedKeyState()
        elif event.key() == QtCore.Qt.Key_D:
            self.rightButtonElement.releasedKeyState()
        elif event.key() == QtCore.Qt.Key_S:
            self.backwardButtonElement.releasedKeyState()
        elif event.key() == QtCore.Qt.Key_A:
            self.leftButtonElement.releasedKeyState()
        event.accept()

    def settingsClicked(self):
        self.controlPanelStack.goToSettings()

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
