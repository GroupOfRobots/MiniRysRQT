# This Python file uses the following encoding: utf-8
import logging
import math
import os
import sys
import threading
import time
from enum import Enum


from python_qt_binding import QtCore, QtWidgets
from python_qt_binding.QtCore import QSize, QFile, Qt, QPoint
from python_qt_binding.QtGui import QPainter, QBrush, QPen
from python_qt_binding.QtWidgets import QPushButton, QMainWindow, QWidget
from ament_index_python import get_resource
from python_qt_binding import loadUi

from .button import Button

class ControlPanelWidget(QWidget):
    def __init__(self,node, plugin=None):
        super(ControlPanelWidget, self).__init__()

        _, package_path = get_resource('packages', 'control_panel')
        ui_file = os.path.join(package_path, 'share', 'control_panel', 'resource', 'control_panel.ui')
        loadUi(ui_file, self)

        self.setFocusPolicy(Qt.ClickFocus)
        self.setFocus()

        self.defineButtons()

        # self.show()

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

    def defineButtons(self):
        self.forwardButton = Button(self.findChild(QPushButton, 'forwardButton'))
        self.rightButton = Button(self.findChild(QPushButton, 'rightButton'))
        self.backwardButton = Button(self.findChild(QPushButton, 'backwardButton'))
        self.leftButton = Button(self.findChild(QPushButton, 'leftButton'))

    def resizeEvent(self, event):
        print("resize")
        # print(self.backwardButton.size().width())
        # self.setIconSize()

    def setIconSize(self):
        width = self.backwardButton.size().width() * 0.9
        height = self.backwardButton.size().height() * 0.9
        self.forwardButton.resizeIcon(width,height)
        self.leftButton.resizeIcon(width,height)
        self.rightButton.resizeIcon(width,height)
        self.backwardButton.resizeIcon(width,height)