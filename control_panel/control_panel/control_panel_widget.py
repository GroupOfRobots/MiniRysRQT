# This Python file uses the following encoding: utf-8
import os

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from python_qt_binding import QtCore
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import QPushButton, QWidget
from ament_index_python import get_resource
from python_qt_binding import loadUi

# from elements.button import Button

# from .elements.button import Button
from . import button
from .elements.button import Button


class ControlPanelWidget(QWidget):
    def __init__(self, node, plugin=None):
        super(ControlPanelWidget, self).__init__()

        _, package_path = get_resource('packages', 'control_panel')
        ui_file = os.path.join(package_path, 'share', 'control_panel', 'resource', 'control_panel.ui')
        loadUi(ui_file, self)

        self.setFocusPolicy(Qt.ClickFocus)
        self.setFocus()

        self.defineButtons()

        node = rclpy.create_node('emulate_kobuki_node')

        self.pub  = node.create_publisher(String, 'topic', 10)

        self.hello_str = String()
        self.hello_str.data = 'hello world'
        self.pub.publish(self.hello_str)

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return
        key = event.key()
        if key == QtCore.Qt.Key_W:
            self.forwardButton.pressedKeyState()
        elif event.key() == QtCore.Qt.Key_D:
            self.rightButton.pressedKeyState()
        elif event.key() == QtCore.Qt.Key_S:
            self.backwardButton.pressedKeyState()
        elif event.key() == QtCore.Qt.Key_A:
            self.leftButton.pressedKeyState()
        event.accept()

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return
        key = event.key()
        if key == QtCore.Qt.Key_W:
            self.forwardButton.releasedKeyState()
        elif event.key() == QtCore.Qt.Key_D:
            self.rightButton.releasedKeyState()
        elif event.key() == QtCore.Qt.Key_S:
            self.backwardButton.releasedKeyState()
        elif event.key() == QtCore.Qt.Key_A:
            self.leftButton.releasedKeyState()
        event.accept()

    def settingsClicked(self):
        parent=self.parent()
        parent.setCurrentIndex(1)

    def buttonClicked(self):
        self.pub.publish(self.hello_str)

    def defineButtons(self):
        self.forwardButtonElement = Button(self.findChild(QPushButton, 'forwardButton'))
        self.rightButtonElement = Button(self.findChild(QPushButton, 'rightButton'))
        self.backwardButtonElement = Button(self.findChild(QPushButton, 'backwardButton'))
        self.leftButtonElement = Button(self.findChild(QPushButton, 'leftButton'))
        # self.settingsButton = Button(self.findChild(QPushButton,'settingsButton'))
        # self.settingsButton = Button(self.findChild(QPushButton,'settingsButton1111'))
        # print(self.settingsButton)

        # self.forwardButton.clicked.connect(self.settingsClicked)
        self.settingsButton.clicked.connect(self.settingsClicked)
        self.forwardButton.clicked.connect(self.buttonClicked)



    def resizeEvent(self, event):
        print("resize")
        # print(self.backwardButton.size().width())
        self.setIconSize()

    def setIconSize(self):
        width = self.backwardButton.size().width() * 0.9
        height = self.backwardButton.size().height() * 0.9
        self.forwardButtonElement.resizeIcon(width, height)
        self.leftButtonElement.resizeIcon(width, height)
        self.rightButtonElement.resizeIcon(width, height)
        self.backwardButtonElement.resizeIcon(width, height)
