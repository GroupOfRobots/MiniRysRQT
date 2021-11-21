# This Python file uses the following encoding: utf-8
import math
import os
import sys

from python_qt_binding import QtCore, QtWidgets
from python_qt_binding.QtCore import QSize, QFile, Qt, QPoint
from python_qt_binding.QtGui import QPainter, QBrush, QPen
from python_qt_binding.QtWidgets import QPushButton, QMainWindow, QWidget
from ament_index_python import get_resource
from python_qt_binding import loadUi

class Joystick1(QWidget):
    def __init__(self,node, plugin=None):
        super(Joystick1, self).__init__()

        _, package_path = get_resource('packages', 'control_panel')
        ui_file = os.path.join(package_path, 'share', 'control_panel', 'resource', 'joystick.ui')
        loadUi(ui_file, self)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setPen(QPen(Qt.black, 5, Qt.SolidLine))

        self.paintJoystickBoundary(painter)
        self.paintJoystick(painter)

    def paintJoystickBoundary(self, painter):
        width = self.width()
        height = self.height()
        x = int(width * 0.05)
        y = int(height * 0.05)
        rx = int(0.9 * width)
        ry = int(0.9 * height)
        painter.setPen(QPen(Qt.black, 5, Qt.SolidLine))
        painter.setBrush(QBrush(Qt.red, Qt.SolidPattern))
        painter.drawEllipse(x, y, rx, ry)

    def paintJoystick(self, painter):
        width = self.width()
        height = self.height()
        rx = int(0.2 * width)
        ry = int(0.2 * height)
        x = int(self.joystickPosition.x() - 0.5 * rx)
        y = int(self.joystickPosition.y() - 0.5 * ry)
        painter.setBrush(QBrush(Qt.cyan, Qt.SolidPattern))
        painter.drawEllipse(x, y, rx, ry)

    def mouseReleaseEvent(self,event):
        self.joystickPosition = QPoint(self.width() * 0.5, self.height() * 0.5)
        self.update()


    def mouseMoveEvent(self, event):
        print("mouseMoveEvent")
        self.joystickPosition = event.pos()
        print(self.joystickPosition)
        x = self.joystickPosition.x()
        y = self.joystickPosition.y()
        h = self.width() * 0.5
        k = self.height() * 0.5
        rx = self.width() * 0.45 - 0.1 * self.width()
        ry = self.height() * 0.45 - 0.1 * self.height()

        if ((x - h) ** 2 / rx ** 2 + (y - k) ** 2 / ry ** 2 <= 1):
            self.update()


    def resizeEvent(self, event):
        # print("event")
        self.joystickPosition = QPoint(self.width() * 0.5, self.height() * 0.5)


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
