# This Python file uses the following encoding: utf-8
import math
import threading
import time

from geometry_msgs.msg import Twist
from python_qt_binding import QtCore
from python_qt_binding.QtCore import Qt, QPoint
from python_qt_binding.QtGui import QPainter, QBrush, QPen
from shared.base_widget.base_widget import BaseWidget
from shared.enums import ControlKeyEnum
from shared.enums import PackageNameEnum

from .services.engines_value_service import EnginesValueService
from .services.key_press_service import KeyPressService


class JoystickWidget(BaseWidget):
    BOUNDARY_RADIUS = 0.45
    BOUNDARY_DIAMETER = BOUNDARY_RADIUS * 2
    JOYSTICK_RADIUS = 0.1
    JOYSTICK_DIAMETER = JOYSTICK_RADIUS * 2
    MARGIN = (1 - BOUNDARY_DIAMETER) / 2
    MARGIN_HORIZONTAL = 2 * MARGIN

    def __init__(self, stack=None, node=None):
        super(JoystickWidget, self).__init__(stack, PackageNameEnum.Joystick)

        self.node = node

        self.keyPressedThread = threading.Thread()

        self.setMouseTracking(False)

        self.setRobotOnScreen()

        self.comboBox.currentIndexChanged.connect(self.setRobotOnScreen)
        self.joystickWidget.mouseMoveEvent = self.joystickWidgetMouseMove

        self.settingsButton.clicked.connect(self.settingsClicked)

        self.publisher = self.node.create_publisher(Twist, self.namespace + '/cmd_vel', 10)

    def settingsClicked(self):
        self.stack.goToSettings(self.dataFilePath)

    def joystickWidgetMouseMove(self, event):
        self.joystickPosition = event.pos()
        x = self.joystickPosition.x()
        y = self.joystickPosition.y()

        cartesianPositionX = x - self.joystickWidget.width() * 0.5
        cartesianPositionY = self.joystickWidget.height() * 0.5 - y

        angle = math.radians(math.atan2(cartesianPositionY, cartesianPositionX) / math.pi * 180)
        elipseR = self.calculateRadiusOfJoystickBoundary(angle)
        joystickR = math.sqrt(cartesianPositionX ** 2 + cartesianPositionY ** 2)

        # print(angle)
        linear, angular = self.enginesValueService.calculateTwistEnginesValue(angle, joystickR, elipseR)
        # print(linear, angular, "aaa")

        msg = Twist()
        msg.linear.y = float(linear)
        msg.angular.z = float(angular)
        self.publisher.publish(msg)
        # print(angle, ellipseR, joystickR)

        self.updateJoystickPosition(x, y)

        if self.checkIfPointIsInEllipse(x, y):
            self.update()

    def initializeRobotSettings(self):
        self.controlKeys = self.data.get('controlKeys')
        self.keyPressService = KeyPressService(self.controlKeys)
        self.enginesValueService = EnginesValueService(self.data.get("joystick", {}))

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
        msg = Twist()
        msg.linear.y = float(0)
        msg.angular.z = float(0)
        self.publisher.publish(msg)
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

    def calculateNewJoystickPosition(self):
        x = self.joystickPosition.x() + self.keyPressService.xMove
        y = self.joystickPosition.y() + self.keyPressService.yMove

        return x, y

    def updateJoystickPosition(self, x, y):
        self.joystickPosition.setX(x)
        self.joystickPosition.setY(y)
        self.update()

    def shouldUpdateJoystick(self, x, y):
        return self.checkIfPointIsInEllipse(x, y) and not (
                ControlKeyEnum.STABLE in self.keyPressService.pressedKeys and len(
            self.keyPressService.pressedKeys) == 1)

    def calculateRadiusOfJoystickBoundary(self, angle):
        ellipseR = self.innerEllipseRx * self.innerEllipseRy * 0.25 / (math.sqrt(
            (self.innerEllipseRx * 0.5) ** 2 * math.sin(angle) ** 2 + (
                    self.innerEllipseRy * 0.5) ** 2 * math.cos(angle) ** 2))
        return ellipseR

    def getAngle(self, cartesianPositionY, cartesianPositionX):
        angle = math.radians(math.atan2(cartesianPositionY, cartesianPositionX) / math.pi * 180)
        return angle

    def calculateKeyPressed(self):
        while self.keyPressedFlag:
            x, y = self.calculateNewJoystickPosition()

            if self.shouldUpdateJoystick(x, y):
                cartesianPositionX = x - self.joystickWidget.width() * 0.5
                cartesianPositionY = self.joystickWidget.height() * 0.5 - y

                angle = math.radians(math.atan2(cartesianPositionY, cartesianPositionX) / math.pi * 180)

                elipseR = self.calculateRadiusOfJoystickBoundary(angle)
                joystickR = math.sqrt(cartesianPositionX ** 2 + cartesianPositionY ** 2)

                leftEngine, rightEngine = self.enginesValueService.calculateMotorCommandEnginesValue(angle, joystickR, elipseR)
                linear, angular = self.enginesValueService.calculateTwistEnginesValue(angle, joystickR, elipseR)
                # print(linear, angular, "aaa")

                msg = Twist()
                msg.linear.y = float(linear)
                msg.angular.z = float(angular)
                self.publisher.publish(msg)
                # print(angle, ellipseR, joystickR)

                self.updateJoystickPosition(x, y)
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
        if not self.keyPressService.onKeyPressed(event):
            return

        if not self.keyPressedThread.is_alive():
            self.keyPressedFlag = True
            self.keyPressedThread = threading.Thread(target=self.calculateKeyPressed, args=())
            self.keyPressedThread.start()

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return

        if not self.keyPressService.onKeyReleaseEvent(event):
            return

        if len(self.keyPressService.pressedKeys) == 0:
            self.keyPressedFlag = False
            self.keyPressedThread.join()
            self.returnToCenter()

        event.accept()
