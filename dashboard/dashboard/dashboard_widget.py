# This Python file uses the following encoding: utf-8
import math
from collections import namedtuple

from minirys_msgs.msg import BatteryStatus, AngularPose
from nav_msgs.msg import Odometry
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QPainter, QPen
from rclpy.node import Node
from sensor_msgs.msg import Range
from shared.base_widget.base_widget import BaseWidget
from shared.enums import PackageNameEnum
from std_msgs.msg import Float32


class DashboardWidget(BaseWidget):
    def __init__(self, stack=None, node=Node):
        super(DashboardWidget, self).__init__(stack, PackageNameEnum.Dashboard, node=node)

        self.predefineSubscribers()

        self.setRobotOnScreen()

        self.angularPosition = 0

        self.batteryUnderVoltageFlag = False

        self.angleWidget.paintEvent = self.paintRobotAngle

    def predefineSubscribers(self):
        self.subscriberParams = [
            SubscriberParam(None, Range, '/internal/distance_0', self.topDistanceSensorCallback),
            SubscriberParam(None, Range, '/internal/distance_1', self.bottomDistanceSensorCallback),
            SubscriberParam(None, Range, '/internal/distance_2', self.rightDistanceSensorCallback),
            SubscriberParam(None, Range, '/internal/distance_3', self.backDistanceSensorCallback),
            SubscriberParam(None, Range, '/internal/distance_4', self.frontDistanceSensorCallback),
            SubscriberParam(None, Range, '/internal/distance_5', self.leftDistanceSensorCallback),
            SubscriberParam(None, BatteryStatus, '/internal/battery_status', self.batteryCallback),
            SubscriberParam(None, Float32, '/internal/temperature_cpu', self.temperatureCpuCallback),
            SubscriberParam(None, Float32, '/internal/temperature_main', self.temperatureMainCallback),
            SubscriberParam(None, AngularPose, '/internal/angular_pose', self.angularPoseCallback),
            SubscriberParam(None, Odometry, '/odom', self.odometryVelocityCallback)]

    def resetSubscribers(self):
        for subscriberParam in self.subscriberParams:
            subscriber = subscriberParam.subscriber
            if subscriber is not None:
                self.node.destroy_subscription(subscriber)

    def initializeRobotSettings(self):
        self.initializeSubscribers()

    def initializeSubscribers(self):
        self.resetSubscribers()
        for index, subscriberParam in enumerate(self.subscriberParams):
            subscriberInstance = self.node.create_subscription(subscriberParam.messageType,
                                                               self.namespace + subscriberParam.topic,
                                                               subscriberParam.callback, 10)
            subscriberParam = subscriberParam._replace(subscriber=subscriberInstance)
            self.subscriberParams[index] = subscriberParam

    def batteryCallback(self, event):
        voltageCell1 = float('{:.3f}'.format(event.voltage_cell1))
        voltageCell2 = float('{:.3f}'.format(event.voltage_cell2))
        voltageCell3 = float('{:.3f}'.format(event.voltage_cell3))

        self.voltageCell1LcdUI.display(voltageCell1)
        self.voltageCell2LcdUI.display(voltageCell2)
        self.voltageCell3LcdUI.display(voltageCell3)

        underVoltageError = event.undervoltage_error
        underVoltageWarning = event.undervoltage_warning

        if underVoltageError and not self.batteryUnderVoltageFlag:
            self.previousStyleSheet = self.batteryGroupBoxUI.styleSheet()
            self.batteryGroupBoxUI.setStyleSheet(
                self.previousStyleSheet + 'QGroupBox { background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 red, stop: 1 pink);}')
            self.batteryGroupBoxUI.setToolTip('<h2>ERROR</h2><div>Battery voltage is too low</div>')
            self.batteryGroupBoxUI.setToolTipDuration(5000)
            self.batteryUnderVoltageFlag = True

        elif underVoltageWarning and not self.batteryUnderVoltageFlag:
            self.previousStyleSheet = self.batteryGroupBoxUI.styleSheet()
            self.batteryGroupBoxUI.setStyleSheet(
                self.previousStyleSheet + 'QGroupBox { background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 orange, stop: 1 yellow);}')
            self.batteryGroupBoxUI.setToolTip(
                '<h2>WARNING</h2><div>Battery voltage is too low</div>')
            self.batteryGroupBoxUI.setToolTipDuration(5000)
            self.batteryUnderVoltageFlag = True

        elif self.batteryUnderVoltageFlag and not underVoltageError and not underVoltageWarning:
            self.batteryUnderVoltageFlag = False
            self.batteryGroupBoxUI.setToolTip('')
            self.batteryGroupBoxUI.setStyleSheet(self.previousStyleSheet)

    def temperatureCpuCallback(self, event):
        self.temperatureCpuLcdUI.display(event.data)

    def temperatureMainCallback(self, event):
        self.temperatureMainLcdUI.display(event.data)

        # 0 - top
        # 1 - bottom
        # 2 - right
        # 3 - back
        # 4 - front
        # 5 - left

    def topDistanceSensorCallback(self, event):
        self.topDistanceLcdUI.display(event.range)

    def bottomDistanceSensorCallback(self, event):
        self.bottomDistanceLcdUI.display(event.range)

    def rightDistanceSensorCallback(self, event):
        self.rightDistanceLcdUI.display(event.range)

    def backDistanceSensorCallback(self, event):
        self.backDistanceLcdUI.display(event.range)

    def frontDistanceSensorCallback(self, event):
        self.frontDistanceLcdUI.display(event.range)

    def leftDistanceSensorCallback(self, event):
        self.leftDistanceLcdUI.display(event.range)

    def backSensorCallback(self, event):
        self.backDistanceLcdUI.display(event.range)

    def paintRobotAngle(self, event):
        painter = QPainter()
        painter.begin(self.angleWidget)
        painter.setPen(QPen(Qt.black, 5, Qt.SolidLine))

        width = self.angleWidget.width()
        height = self.angleWidget.height()
        x1 = self.angleWidget.x() + round(width * 0.5)
        y1 = self.angleWidget.y() + round(height * 0.9)
        xMove = int(math.sin(self.angularPosition) * 50)

        yMove = int(math.cos(self.angularPosition) * 50)

        painter.drawLine(x1, y1, x1 - xMove, y1 - yMove)

        painter.setPen(QPen(Qt.gray, 5, Qt.DotLine))

        x1Bottom = self.angleWidget.x() + round(width * 0.2)
        x2Bottom = self.angleWidget.x() + round(width * 0.8)
        painter.drawLine(x1Bottom, y1, x2Bottom, y1)

    def angularPoseCallback(self, event):
        self.angularPosition = event.angular_position
        self.angleWidget.update()

    def odometryVelocityCallback(self, event):
        twist = event.twist.twist
        linear = twist.linear
        angular = twist.angular

        self.yLinearVelcocityLcdUI.display(linear.y)
        self.zAngularVelcocityLcdUI.display(angular.z)


SubscriberParam = namedtuple('SubscriberParam', ["subscriber", "messageType", "topic", "callback"])
