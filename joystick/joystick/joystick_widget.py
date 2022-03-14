# This Python file uses the following encoding: utf-8
import json
import os
import threading
import time

from ament_index_python import get_resource
from python_qt_binding import QtCore
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QPoint
from python_qt_binding.QtGui import QPainter, QBrush, QPen
from shared.base_widget.base_widget import BaseWidget
from shared.enums import ControlKeyEnum


class JoystickWidget(BaseWidget):
    BOUNDARY_RADIUS = 0.45
    BOUNDARY_DIAMETER = BOUNDARY_RADIUS * 2
    JOYSTICK_RADIUS = 0.1
    JOYSTICK_DIAMETER = JOYSTICK_RADIUS * 2
    MARGIN = 0.05
    MARGIN_HORIZONTAL = 2 * MARGIN

    def __init__(self,stack=None):
        super(JoystickWidget, self).__init__()
        BaseWidget.__init__(self, stack)

        self.stack=stack

        _, package_path = get_resource('packages', 'joystick')
        ui_file = os.path.join(package_path, 'share', 'joystick', 'resource', 'joystick.ui')
        loadUi(ui_file, self)

        self.pressedKeys = []
        self.xMove = 0
        self.yMove = 0
        self.keyPressedThread = threading.Thread()

        self.initializeRobotsOptions()
        self.initializeSettings(self.comboBox.currentData()['filePath'])

    def initializeSettings(self, filePath):
        dataFile = open(filePath)
        data = json.load(dataFile)
        dataFile.close()
        self.controlKeys = data['controlKeys']

        for key in self.controlKeys:
            controlValue = self.controlKeys[key].upper()
            self.controlKeys[key] = QtCore.Qt.Key(ord(controlValue))

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setPen(QPen(Qt.black, 5, Qt.SolidLine))
        pos = self.joystickWidget.pos()
        rect = self.joystickWidget.rect()
        painter.setClipRect(pos.x(), pos.y(), rect.width(), rect.height())

        self.paintJoystickBoundary(painter)
        self.paintJoystick(painter)

    def paintJoystickBoundary(self, painter):
        width = self.joystickWidget.width()
        height = self.joystickWidget.height()
        startX = self.joystickWidget.pos().x()
        startY = self.joystickWidget.pos().y()
        x = startX + int(width * self.MARGIN)
        y = startY + int(height * self.MARGIN)
        rx = int(self.BOUNDARY_DIAMETER * width)
        ry = int(self.BOUNDARY_DIAMETER * height)

        painter.setPen(QPen(Qt.black, 5, Qt.SolidLine))
        painter.setBrush(QBrush(Qt.red, Qt.SolidPattern))
        painter.drawEllipse(x, y, rx, ry)

    def paintJoystick(self, painter):
        width = self.joystickWidget.width()
        height = self.joystickWidget.height()
        startX = self.joystickWidget.pos().x()
        startY = self.joystickWidget.pos().y()
        rx = int(self.JOYSTICK_DIAMETER * width)
        ry = int(self.JOYSTICK_DIAMETER * height)
        x = startX + int(self.joystickPosition.x() - 0.5 * rx)
        y = startY + int(self.joystickPosition.y() - 0.5 * ry)
        painter.setBrush(QBrush(Qt.cyan, Qt.SolidPattern))
        painter.drawEllipse(x, y, rx, ry)

    def returnToCenter(self):
        self.joystickPosition = QPoint(self.joystickWidget.width() * 0.5, self.joystickWidget.height() * 0.5)
        self.update()

    def mouseReleaseEvent(self, event):
        self.returnToCenter()

    def mouseMoveEvent(self, event):
        self.joystickPosition = event.pos()
        x = self.joystickPosition.x()
        y = self.joystickPosition.y()

        if self.checkIfPointIsInEllipse(x, y):
            self.update()

    def resizeEvent(self, event):
        self.joystickPosition = QPoint(self.joystickWidget.width() * 0.5, self.joystickWidget.height() * 0.5)

    def calculateKeyPressed(self):
        while self.keyPressedFlag:
            x = self.joystickPosition.x() + self.xMove
            y = self.joystickPosition.y() + self.yMove

            if self.checkIfPointIsInEllipse(x, y):
                self.joystickPosition.setY(y)
                self.joystickPosition.setX(x)

                self.update()
                time.sleep(0.001)
            else:
                time.sleep(0.1)

    def checkIfPointIsInEllipse(self, x, y):
        h = self.joystickWidget.width() * 0.5
        k = self.joystickWidget.height() * 0.5
        # half of drawEllipse radius minus boundary margins from both sides
        rx = self.joystickWidget.width() * self.BOUNDARY_RADIUS - self.MARGIN_HORIZONTAL * self.joystickWidget.width()
        ry = self.joystickWidget.height() * self.BOUNDARY_RADIUS - self.MARGIN_HORIZONTAL * self.joystickWidget.height()

        return (x - h) ** 2 / rx ** 2 + (y - k) ** 2 / ry ** 2 <= 1

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return
        # key = QtCore.Qt.Key(event.key())
        key = event.key()

        if key == self.controlKeys[ControlKeyEnum.FORWARD]:
            self.pressedKeys.append(ControlKeyEnum.FORWARD)
            self.yMove = -1
        elif key == self.controlKeys[ControlKeyEnum.RIGHT]:
            self.pressedKeys.append(ControlKeyEnum.RIGHT)
            self.xMove = 1
        elif key == self.controlKeys[ControlKeyEnum.BACKWARD]:
            self.pressedKeys.append(ControlKeyEnum.BACKWARD)
            self.yMove = 1
        elif key == self.controlKeys[ControlKeyEnum.LEFT]:
            self.pressedKeys.append(ControlKeyEnum.LEFT)
            self.xMove = -1
        elif key == self.controlKeys[ControlKeyEnum.STABLE]:
            self.pressedKeys.append(ControlKeyEnum.STABLE)
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
        if key == self.controlKeys[ControlKeyEnum.FORWARD]:
            self.pressedKeys.remove(ControlKeyEnum.FORWARD)
            if ControlKeyEnum.BACKWARD in self.pressedKeys:
                self.yMove = 1
            else:
                self.yMove = 0
        elif key == self.controlKeys[ControlKeyEnum.RIGHT]:
            self.pressedKeys.remove(ControlKeyEnum.RIGHT)
            if ControlKeyEnum.LEFT in self.pressedKeys:
                self.xMove = -1
            else:
                self.xMove = 0
        elif key == self.controlKeys[ControlKeyEnum.BACKWARD]:
            self.pressedKeys.remove(ControlKeyEnum.BACKWARD)
            if ControlKeyEnum.FORWARD in self.pressedKeys:
                self.yMove = -1
            else:
                self.yMove = 0
        elif key == self.controlKeys[ControlKeyEnum.LEFT]:
            self.pressedKeys.remove(ControlKeyEnum.LEFT)
            if ControlKeyEnum.RIGHT in self.pressedKeys:
                self.xMove = 1
            else:
                self.xMove = 0
        elif key == QtCore.Qt.Key_Q:
            self.pressedKeys.remove(ControlKeyEnum.STABLE)
        else:
            event.accept()
            return

        if len(self.pressedKeys) == 0:
            self.keyPressedFlag = False
            self.keyPressedThread.join()
            self.returnToCenter()

        event.accept()
