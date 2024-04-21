# This Python file uses the following encoding: utf-8
import math
import threading
import time

from python_qt_binding import QtCore
from python_qt_binding.QtCore import Qt, QPoint, QEvent
from python_qt_binding.QtGui import QPainter, QBrush, QPen
from shared.base_widget.base_widget import BaseWidget
from shared.enums import PackageNameEnum, ControlKeyEnum
from shared.publishers.balance_publisher import BalancePublisher
from shared.publishers.bool_publisher import BoolPublisher

from .services.engines_value_service import EnginesValueService
from .services.key_press_service import KeyPressService
from .services.message_service import MessageService


class JoystickWidget(BaseWidget):
    BOUNDARY_RADIUS = 0.45
    BOUNDARY_DIAMETER = BOUNDARY_RADIUS * 2
    JOYSTICK_RADIUS = 0.1
    JOYSTICK_DIAMETER = JOYSTICK_RADIUS * 2
    MARGIN = (1 - BOUNDARY_DIAMETER) / 2
    MARGIN_HORIZONTAL = 2 * MARGIN

    def __init__(self, stack=None, node=None):
        super(JoystickWidget, self).__init__(stack, PackageNameEnum.Joystick, node=node)

        self.balancePublisher = BalancePublisher(self.balanceCheckBoxUI, self.node, self)
        self.servoPublisher = BoolPublisher(self.servoCheckBoxUI, self.node)

        self.messageService = MessageService(self.messageTypeComboBoxUI, self.node)

        self.keyPressedThread = threading.Thread()

        self.setMouseTracking(False)

        self.setRobotOnScreen()

        self.comboBox.currentIndexChanged.connect(self.setRobotOnScreen)
        self.joystickWidget.mouseMoveEvent = self.joystickWidgetMouseMove
        self.debounceTimestamp = time.time()

        self.previousAngle = 0
        self.previousPreviousAngle = 0

    def joystickWidgetMouseMove(self, event):
        self.joystickPosition = event.pos()
        x = self.joystickPosition.x()
        y = self.joystickPosition.y()

        timestamp = time.time()

        if self.checkIfPointIsInEllipse(x, y):
            self.updateJoystickPosition(x, y)
            if timestamp - self.debounceTimestamp > 0.01:
                self.debounceTimestamp = time.time()
                self.updateRobot(x, y)
                self.update()
        else:
            newX, newY, _ = self.findNearestPoint(x, y)
            self.updateJoystickPosition(newX, newY)
            if timestamp - self.debounceTimestamp > 0.01:
                self.debounceTimestamp = time.time()
                self.updateRobot(x, y)
                self.update()

    def initializeRobotSettings(self):
        self.controlKeys = self.data.get('controlKeys')
        self.keyPressService = KeyPressService(self.controlKeys)
        self.enginesValueService = EnginesValueService(self.data.get("joystick", {}))

        pidData = self.data.get('pid', {})
        self.balancePublisher.setup(self.namespace, '/balance_mode', pidData)

        self.servoPublisher.setTopic(self.namespace, '/servo_status')
        self.messageService.setup(self.namespace)

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

        self.stopRobot()
        self.update()

    def stopRobot(self):
        if self.messageTypeComboBoxUI.currentIndex() == 0:
            self.messageService.publishTwist(0.0, 0.0)
        else:
            self.messageService.publishMotorCommand(0.0, 0.0)

    def eventFilter(self, obj, event):
        if event.type() == QEvent.FocusOut:
            if self.keyPressedThread.is_alive():
                self.keyPressedFlag = False
                self.keyPressedThread.join()
            self.keyPressService.pressedKeys = []
            self.returnToCenter()
        return super().eventFilter(obj, event)

    def mouseReleaseEvent(self, event):
        self.returnToCenter()

    def resizeEvent(self, event):
        # stop robot when resizing
        if self.keyPressedThread.is_alive():
            self.keyPressedFlag = False
            self.keyPressedThread.join()
        self.keyPressService.pressedKeys = []
        self.stopRobot()

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

    def shouldCalculateNewJoystickPosition(self, x, y):
        isStableAndAtLeastOneKeyIsPressed = (ControlKeyEnum.STABLE in self.keyPressService.pressedKeys and len(
            self.keyPressService.pressedKeys) >= 2)
        isNotStableAndAtLeastOneKeyIsPressed = (
                (not (ControlKeyEnum.STABLE in self.keyPressService.pressedKeys)) and len(
            self.keyPressService.pressedKeys) >= 1)
        return isStableAndAtLeastOneKeyIsPressed or isNotStableAndAtLeastOneKeyIsPressed

    def calculateRadiusOfJoystickBoundary(self, angle):
        ellipseR = self.innerEllipseRx * self.innerEllipseRy * 0.25 / (math.sqrt(
            (self.innerEllipseRx * 0.5) ** 2 * math.sin(angle) ** 2 + (
                    self.innerEllipseRy * 0.5) ** 2 * math.cos(angle) ** 2))
        return ellipseR

    def getAngle(self, x, y):
        cartesianPositionX, cartesianPositionY = self.getCartesianPosition(x, y)
        angle = math.radians(math.atan2(cartesianPositionY, cartesianPositionX) / math.pi * 180)
        return angle

    def getCartesianPosition(self, x, y):
        cartesianPositionX = x - self.joystickWidget.width() * 0.5
        cartesianPositionY = self.joystickWidget.height() * 0.5 - y
        return cartesianPositionX, cartesianPositionY

    def calculateKeyPressed(self):
        while self.keyPressedFlag:
            x, y = self.calculateNewJoystickPosition()

            if self.shouldUpdateJoystick(x, y):
                self.updateRobot(x, y)

                self.previousAngle = self.getAngle(x, y)

                self.updateJoystickPosition(x, y)
                time.sleep(0.002)
            elif self.shouldCalculateNewJoystickPosition(x, y):
                newX, newY, newAngle = self.findNearestPoint(self.joystickPosition.x() + 2 * self.keyPressService.xMove,
                                                             self.joystickPosition.y() + 2 * self.keyPressService.yMove)

                if abs(abs(newAngle) - abs(self.previousAngle)) < 0.0001 or abs(abs(newAngle) - abs(self.previousPreviousAngle)) < 0.0001:
                    time.sleep(0.1)

                else:
                    self.updateRobot(newX, newY)
                    self.updateJoystickPosition(newX, newY)
                    time.sleep(0.005)
                self.previousPreviousAngle = self.previousAngle
                self.previousAngle = newAngle

    def getJoystickSituation(self, x, y):
        cartesianPositionX, cartesianPositionY = self.getCartesianPosition(x, y)

        angle = self.getAngle(x, y)

        elipseR = self.calculateRadiusOfJoystickBoundary(angle)
        joystickR = math.sqrt(cartesianPositionX ** 2 + cartesianPositionY ** 2)

        return angle, elipseR, joystickR

    def updateRobot(self, x, y):
        angle, elipseR, joystickR = self.getJoystickSituation(x, y)
        if self.messageTypeComboBoxUI.currentIndex() == 0:
            linear, angular = self.enginesValueService.calculateTwistEnginesValue(angle, joystickR, elipseR)
            self.messageService.publishTwist(linear, angular)
        else:
            leftEngine, rightEngine = self.enginesValueService.calculateMotorCommandEnginesValue(angle, joystickR,
                                                                                                 elipseR)
            self.messageService.publishMotorCommand(leftEngine, rightEngine)

    def checkIfPointIsInEllipse(self, x, y):
        h = self.joystickWidget.width() * 0.5
        k = self.joystickWidget.height() * 0.5

        # half of drawEllipse radius minus boundary margins from both sides
        rx = self.joystickWidget.width() * self.BOUNDARY_RADIUS - self.MARGIN_HORIZONTAL * self.joystickWidget.width()
        ry = self.joystickWidget.height() * self.BOUNDARY_RADIUS - self.MARGIN_HORIZONTAL * self.joystickWidget.height()

        return (x - h) ** 2 / rx ** 2 + (y - k) ** 2 / ry ** 2 <= 1

    def findNearestPoint(self, x, y):
        rx, ry = self.getEllipsisAxises()
        h, k = self.getEllipseCenter()
        R = math.sqrt(x ** 2 + y ** 2)
        sinA = (y - k) / R
        cosA = (x - h) / R

        # https://math.stackexchange.com/questions/432902/how-to-get-the-radius-of-an-ellipse-at-a-specific-angle-by-knowing-its-semi-majo
        ellipseRadius = rx * ry / math.sqrt(rx ** 2 * sinA ** 2 + ry ** 2 * cosA ** 2)

        newXCandidate = cosA * ellipseRadius + h
        newYCandidate = sinA * ellipseRadius + k

        newX = math.ceil(newXCandidate) if newXCandidate < h else math.floor(newXCandidate)
        newY = math.ceil(newYCandidate) if newYCandidate < k else math.floor(newYCandidate)

        newAngle = self.getAngle(newXCandidate, newYCandidate)
        return newX, newY, newAngle

    def getEllipseCenter(self):
        h = self.joystickWidget.width() * 0.5
        k = self.joystickWidget.height() * 0.5
        return h, k

    def getEllipsisAxises(self):
        rx = self.joystickWidget.width() * self.BOUNDARY_RADIUS - self.MARGIN_HORIZONTAL * self.joystickWidget.width()
        ry = self.joystickWidget.height() * self.BOUNDARY_RADIUS - self.MARGIN_HORIZONTAL * self.joystickWidget.height()

        return rx, ry

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

    def cleanup(self):
        if self.keyPressedThread.is_alive():
            self.keyPressedFlag = False
            self.keyPressedThread.join()
        self.stopRobot()
