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

class JoystickWidget(QWidget):
    def __init__(self,node, plugin=None):
        super(JoystickWidget, self).__init__()

        _, package_path = get_resource('packages', 'joystick')
        ui_file = os.path.join(package_path, 'share', 'joystick', 'resource', 'joystick.ui')
        loadUi(ui_file, self)

        self.setFocusPolicy(Qt.ClickFocus)
        self.setFocus()

        self.pressedKeys=[]
        self.xMove = 0
        self.yMove = 0
        self.keyPressedThread = threading.Thread()


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

    def returnToCenter(self):
        self.joystickPosition = QPoint(self.width() * 0.5, self.height() * 0.5)
        self.update()

    def mouseReleaseEvent(self,event):
        self.returnToCenter()


    def mouseMoveEvent(self, event):
        self.joystickPosition = event.pos()
        x = self.joystickPosition.x()
        y = self.joystickPosition.y()
        h = self.width() * 0.5
        k = self.height() * 0.5
        rx = self.width() * 0.45 - 0.1 * self.width()
        ry = self.height() * 0.45 - 0.1 * self.height()

        if ((x - h) ** 2 / rx ** 2 + (y - k) ** 2 / ry ** 2 <= 1):
            self.update()


    def resizeEvent(self, event):
        self.joystickPosition = QPoint(self.width() * 0.5, self.height() * 0.5)


    def calculateKeyPressed(self):
        while self.keyPressedFlag:
            x = self.joystickPosition.x() + self.xMove
            y = self.joystickPosition.y() + self.yMove
            h = self.width() * 0.5
            k = self.height() * 0.5
            rx = self.width() * 0.45 - 0.1 * self.width()
            ry = self.height() * 0.45 - 0.1 * self.height()

            if ((x - h) ** 2 / rx ** 2 + (y - k) ** 2 / ry ** 2 <= 1):
                self.joystickPosition.setY(y)
                self.joystickPosition.setX(x)

                self.update()
                time.sleep(0.001)
            else:
                time.sleep(0.1)

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return
        key = event.key()
        if key == QtCore.Qt.Key_W:
            self.pressedKeys.append(KeyDirections.FORWARD)
            self.yMove=-1
        elif event.key() == QtCore.Qt.Key_D:
            self.pressedKeys.append(KeyDirections.RIGHT)
            self.xMove = 1
        elif event.key() == QtCore.Qt.Key_S:
            self.pressedKeys.append(KeyDirections.BACKWARD)
            self.yMove = 1
        elif event.key() == QtCore.Qt.Key_A:
            self.pressedKeys.append(KeyDirections.LEFT)
            self.xMove = -1
        elif event.key() == QtCore.Qt.Key_Q:
            self.pressedKeys.append(KeyDirections.STABLE)
        else:
            event.accept()
            return
        if not self.keyPressedThread.is_alive():
            self.keyPressedFlag = True
            self.keyPressedThread = threading.Thread(target=self.calculateKeyPressed, args=())
            self.keyPressedThread.start()

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return
        key = event.key()
        if key == QtCore.Qt.Key_W:
            self.pressedKeys.remove(KeyDirections.FORWARD)
            if KeyDirections.BACKWARD in self.pressedKeys:
                self.yMove = 1
            else:
                self.yMove = 0
        elif event.key() == QtCore.Qt.Key_D:
            self.pressedKeys.remove(KeyDirections.RIGHT)
            if KeyDirections.LEFT in self.pressedKeys:
                self.xMove = -1
            else:
                self.xMove = 0
        elif event.key() == QtCore.Qt.Key_S:
            self.pressedKeys.remove(KeyDirections.BACKWARD)
            if KeyDirections.FORWARD in self.pressedKeys:
                self.yMove = -1
            else:
                self.yMove = 0
        elif event.key() == QtCore.Qt.Key_A:
            self.pressedKeys.remove(KeyDirections.LEFT)
            if KeyDirections.RIGHT in self.pressedKeys:
                self.xMove = 1
            else:
                self.xMove = 0
        elif event.key() == QtCore.Qt.Key_Q:
            self.pressedKeys.remove(KeyDirections.STABLE)
        else:
            event.accept()
            return

        if len(self.pressedKeys) == 0:
            self.keyPressedFlag = False
            self.keyPressedThread.join()
            self.returnToCenter()

        event.accept()


class KeyDirections(Enum):
    FORWARD='FORWARD'
    RIGHT='RIGHT'
    BACKWARD='BACKWARD'
    LEFT='LEFT'
    STABLE='STABLE'

 #
 # if key == QtCore.Qt.Key_W:
 #            self.goForwardFlag=True
 #            self.forwardThread = threading.Thread(target=self.calculateKeyPressedParams, args=(0,-1,self.goForwardFlag))
 #            self.forwardThread.start()
 #        elif event.key() == QtCore.Qt.Key_D:
 #            self.goRightFlag=True
 #            self.rightThread = threading.Thread(target=self.calculateKeyPressedParams, args=(1,0,self.goRightFlag))
 #            self.rightThread.start()
 #        elif event.key() == QtCore.Qt.Key_S:
 #            self.goBackwardFlag=True
 #            self.backwardThread = threading.Thread(target=self.calculateKeyPressedParams, args=(0,1,self.goBackwardFlag))
 #            self.backwardThread.start()
 #        elif event.key() == QtCore.Qt.Key_A:
 #            self.goLeftFlag=True
 #            self.leftThread = threading.Thread(target=self.calculateKeyPressedParams, args=(-1,0,self.goLeftFlag))
 #            self.leftThread.start()


# if key == QtCore.Qt.Key_W:
#     self.goForwardFlag = False
#     self.forwardThread.join()
# elif event.key() == QtCore.Qt.Key_D:
#     self.goRightFlag = False
#     self.rightThread.join()
# elif event.key() == QtCore.Qt.Key_S:
#     self.goBackwardFlag = False
#     self.backwardThread.join()
# elif event.key() == QtCore.Qt.Key_A:
#     self.goLeftFlag = False
#     self.leftThread.join()