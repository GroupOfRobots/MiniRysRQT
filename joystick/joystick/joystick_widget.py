# This Python file uses the following encoding: utf-8
import json
import math
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

from python_qt_binding.QtWidgets import QWidget

from shared.enums import PackageNameEnum


class JoystickWidget(BaseWidget):
    BOUNDARY_RADIUS = 0.45
    BOUNDARY_DIAMETER = BOUNDARY_RADIUS * 2
    JOYSTICK_RADIUS = 0.1
    JOYSTICK_DIAMETER = JOYSTICK_RADIUS * 2
    MARGIN = (1 - BOUNDARY_DIAMETER) / 2
    MARGIN_HORIZONTAL = 2 * MARGIN

    def __init__(self, stack=None):
        super(JoystickWidget, self).__init__(stack, PackageNameEnum.Joystick)

        self.pressedKeys = []
        self.xMove = 0
        self.yMove = 0
        self.keyPressedThread = threading.Thread()

        self.setMouseTracking(False)

        self.setRobotOnScreen()

        self.comboBox.currentIndexChanged.connect(self.setRobotOnScreen)
        self.joystickWidget.mouseMoveEvent = self.joystickWidgetMouseMove

        self.settingsButton.clicked.connect(self.settingsClicked)

    def settingsClicked(self):
        self.stack.goToSettings(self.dataFilePath)

    def joystickWidgetMouseMove(self, event):
        self.joystickPosition = event.pos()
        x = self.joystickPosition.x()
        y = self.joystickPosition.y()

        if self.checkIfPointIsInEllipse(x, y):
            self.update()

    def initializeRobotSettings(self):
        self.controlKeys = self.data.get('controlKeys')

        for key in self.controlKeys:
            controlValue = self.controlKeys[key].upper()
            self.controlKeys[key] = QtCore.Qt.Key(ord(controlValue))

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setPen(QPen(Qt.black, 5, Qt.SolidLine))
        pos = self.joystickWidget.pos()
        painter.setClipRect(pos.x(), pos.y(), self.joystickWidget.width(), self.joystickWidget.height())

        self.paintJoystickBoundary(painter)
        self.paintJoystick(painter)

    def paintJoystickBoundary(self, painter):
        x = self.startX + int(self.width * self.MARGIN)
        y = self.startY + int(self.height * self.MARGIN)

        painter.setPen(QPen(Qt.black, 5, Qt.SolidLine))
        painter.setBrush(QBrush(Qt.red, Qt.SolidPattern))
        painter.drawEllipse(x, y, self.widgetRx, self.widgetRy)

    def paintJoystick(self, painter):
        x = self.startX + int(self.joystickPosition.x() - 0.5 * self.joystickRx)
        y = self.startY + int(self.joystickPosition.y() - 0.5 * self.joystickRy)

        painter.setBrush(QBrush(Qt.cyan, Qt.SolidPattern))
        painter.drawEllipse(x, y, self.joystickRx, self.joystickRy)

    def returnToCenter(self):
        x = int(self.joystickWidget.width() * 0.5)
        y = int(self.joystickWidget.height() * 0.5)
        self.joystickPosition = QPoint(x, y)
        self.update()

    def mouseReleaseEvent(self, event):
        self.returnToCenter()

    def resizeEvent(self, event):
        x = int(self.joystickWidget.width() * 0.5)
        y = int(self.joystickWidget.height() * 0.5)
        self.joystickPosition = QPoint(x, y)
        self.width = self.joystickWidget.width()
        self.height = self.joystickWidget.height()
        self.startX = self.joystickWidget.pos().x()
        self.startY = self.joystickWidget.pos().y()

        self.widgetRx = int(self.BOUNDARY_DIAMETER * self.width)
        self.widgetRy = int(self.BOUNDARY_DIAMETER * self.height)

        self.joystickRx = int(self.JOYSTICK_DIAMETER * self.width)
        self.joystickRy = int(self.JOYSTICK_DIAMETER * self.height)

        self.innerEllipseRx = self.widgetRx - self.joystickRx
        self.innerEllipseRy = self.widgetRy - self.joystickRy

    def getValue(self, data, fieldName):
        return int(data.get(fieldName, 0))

    def calculateKeyPressed(self):
        while self.keyPressedFlag:
            x = self.joystickPosition.x() + self.xMove
            y = self.joystickPosition.y() + self.yMove

            if self.checkIfPointIsInEllipse(x, y) and not (
                    ControlKeyEnum.STABLE in self.pressedKeys and len(self.pressedKeys) == 1):
                self.joystickPosition.setX(x)
                self.joystickPosition.setY(y)

                cartesianPositionX = x - self.joystickWidget.width() * 0.5
                cartesianPositionY = self.joystickWidget.height() * 0.5 - y

                angle = math.radians(math.atan2(cartesianPositionY, cartesianPositionX) / math.pi * 180)

                print(angle)

                ellipseR = self.innerEllipseRx * self.innerEllipseRy * 0.25 / (math.sqrt(
                    (self.innerEllipseRx * 0.5) ** 2 * math.sin(angle) ** 2 + (
                            self.innerEllipseRy * 0.5) ** 2 * math.cos(angle) ** 2))
                joystickR = math.sqrt(cartesianPositionX ** 2 + cartesianPositionY ** 2)

                print(angle, ellipseR, joystickR)

                joystickData = self.data.get("joystick", {})
                joystickForwardData = joystickData.get("forward", {})
                joystickRightData = joystickData.get("right", {})
                joystickBackwartdData = joystickData.get("backward", {})
                joystickLeftData = joystickData.get("right", {})

                joystickDeflection = joystickR / ellipseR

                leftEngine = 0
                rightEngine = 0

                if angle > 0 and (angle <= math.pi * 0.5):
                    angleFactor = angle / (math.pi * 0.5)
                    leftEngine = self.getValue(joystickRightData, "leftEngine") * (1 - angleFactor) \
                                 + self.getValue(joystickForwardData, "leftEngine") * angleFactor

                    rightEngine = self.getValue(joystickRightData, "rightEngine") * (1 - angleFactor) \
                                  + self.getValue(joystickForwardData, "rightEngine") * angleFactor
                elif angle > math.pi * 0.5:
                    angleFactor = (angle - (math.pi * 0.5)) / (math.pi * 0.5)
                    leftEngine = self.getValue(joystickForwardData, "leftEngine") * (1 - angleFactor) \
                                 + self.getValue(joystickLeftData, "leftEngine") * angleFactor

                    rightEngine = self.getValue(joystickForwardData, "rightEngine") * (1 - angleFactor) \
                                  + self.getValue(joystickLeftData, "rightEngine") * angleFactor


                elif angle < 0 and (angle >= -math.pi * 0.5):
                    angleFactor = abs(angle / (math.pi * 0.5))
                    leftEngine = self.getValue(joystickRightData, "leftEngine") * (1 - angleFactor) \
                                 + self.getValue(joystickBackwartdData, "leftEngine") * angleFactor

                    rightEngine = self.getValue(joystickRightData, "rightEngine") * (1 - angleFactor) \
                                  + self.getValue(joystickBackwartdData, "rightEngine") * angleFactor



                elif angle < (-math.pi * 0.5):
                    angleFactor = abs((angle + (math.pi * 0.5)) / (math.pi * 0.5))
                    leftEngine = self.getValue(joystickBackwartdData, "leftEngine") * (1 - angleFactor) \
                                 + self.getValue(joystickLeftData, "leftEngine") * angleFactor

                    rightEngine = self.getValue(joystickBackwartdData, "rightEngine") * (1 - angleFactor) \
                                  + self.getValue(joystickLeftData, "rightEngine") * angleFactor

                print("engine", leftEngine, rightEngine)
                print("engine", leftEngine*joystickDeflection, rightEngine*joystickDeflection)
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
