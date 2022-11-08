# This Python file uses the following encoding: utf-8
import math
import os
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QPainter, QPen
from ament_index_python import get_resource
from python_qt_binding import loadUi
from shared.base_widget.base_widget import BaseWidget

from rclpy.node import Node
from minirys_msgs.msg import BatteryStatus, AngularPose
from sensor_msgs.msg import Range

from std_msgs.msg import Float32, String

import time
from collections import namedtuple


class DashboardWidget(BaseWidget):
    def __init__(self, stack=None, node=Node):
        super(DashboardWidget, self).__init__()

        self.node = node
        self.predefineSubscribers()


        self.test = None
        self.setRobotOnScreen()

        self.angularPosition = 0
        self.sign = 1

        self.batteryUnderVoltageFlag = False

        self.angleWidget.paintEvent = self.paintRobotAngle

        self.destroyed.connect(DashboardWidget.onDestroyed)

    def loadUI(self):
        _, packagePath = get_resource('packages', 'dashboard')
        uiFile = os.path.join(packagePath, 'share', 'dashboard', 'resource', 'dashboard.ui')
        loadUi(uiFile, self)

    def predefineSubscribers(self):
        self.subscriberParams = [
            SubscriberParam(None, BatteryStatus, '/internal/battery_status', self.batteryCallback),
            SubscriberParam(None, Range, '/internal/distance_0', self.frontSensorCallback),
            SubscriberParam(None, Range, '/internal/distance_1', self.backSensorCallback),
            SubscriberParam(None, Float32, '/internal/temperature_cpu', self.temperatureCpuCallback),
            SubscriberParam(None, Float32, '/internal/temperature_main', self.temperatureMainCallback),
            SubscriberParam(None, AngularPose, '/internal/angular_pose', self.angularPoseCallback)]

    def resetSubscribers(self):
        for subscriberParam in self.subscriberParams:

            subscriber = subscriberParam.subscriber
            if subscriber is not None:
                self.node.destroy_subscription(subscriber)

    def initializeSubscribers(self):
        self.resetSubscribers()
        for index, subscriberParam in enumerate(self.subscriberParams):
            subscriberInstance = self.node.create_subscription(subscriberParam.messageType,
                                                               self.namespace + subscriberParam.topic,
                                                               subscriberParam.callback, 10)
            subscriberParam = subscriberParam._replace(subscriber=subscriberInstance)
            self.subscriberParams[index]=subscriberParam

    def log(self, event):
        print("log")

    def initializeRobotSettings(self):
        print('initializeSettings')
        self.initializeSubscribers()

    def batteryCallback(self, event):
        voltageCell1 = float('{:.3f}'.format(event.voltage_cell1))
        voltageCell2 = float('{:.3f}'.format(event.voltage_cell2))
        voltageCell3 = float('{:.3f}'.format(event.voltage_cell3))

        self.voltageCell1Lcd.display(voltageCell1)
        self.voltageCell2Lcd.display(voltageCell2)
        self.voltageCell3Lcd.display(voltageCell3)

        underVoltageError = event.undervoltage_error
        underVoltageWarning = event.undervoltage_warning

        if underVoltageError and not self.batteryUnderVoltageFlag:
            self.previousStyleSheet = self.batteryGroupBox.styleSheet()
            self.batteryGroupBox.setStyleSheet(
                self.previousStyleSheet + 'QGroupBox { background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 red, stop: 1 pink);}')
            self.batteryGroupBox.setToolTip('<h2>ERROR</h2><div>Battery voltage is too low</div>')
            self.batteryGroupBox.setToolTipDuration(5000)
            self.batteryUnderVoltageFlag = True

        elif underVoltageWarning and not self.batteryUnderVoltageFlag:
            self.previousStyleSheet = self.batteryGroupBox.styleSheet()
            self.batteryGroupBox.setStyleSheet(
                self.previousStyleSheet + 'QGroupBox { background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 orange, stop: 1 yellow);}')
            self.batteryGroupBox.setToolTip(
                '<h2>WARNING</h2><div>Battery voltage is too low</div>')
            self.batteryGroupBox.setToolTipDuration(5000)
            self.batteryUnderVoltageFlag = True

        elif self.batteryUnderVoltageFlag and not underVoltageError and not underVoltageWarning:
            self.batteryUnderVoltageFlag = False
            self.batteryGroupBox.setToolTip('')
            self.batteryGroupBox.setStyleSheet(self.previousStyleSheet)

    def temperatureCpuCallback(self, event):
        self.temperatureCpuLcd.display(event.data)

    def temperatureMainCallback(self, event):
        self.temperatureMainLcd.display(event.data)

    def frontSensorCallback(self, event):
        self.frontDistanceLcd.display(event.range)

    def backSensorCallback(self, event):
        # print(event)
        range = int((event.range * 1000))
        self.backDistanceLcd.display(event.range)

    def paintRobotAngle(self, event):
        painter = QPainter()
        painter.begin(self.angleWidget)
        pos = self.angleWidget.pos()
        painter.setPen(QPen(Qt.black, 5, Qt.SolidLine))
        # painter.setBrush(QBrush(Qt.red, Qt.SolidPattern))

        pos1 = self.angleWidget.mapToGlobal(pos)
        width = self.angleWidget.width()
        height = self.angleWidget.height()
        x1 = self.angleWidget.x() + round(width * 0.5)
        y1 = self.angleWidget.y() + round(height * 0.9)
        xMove = int(math.sin(self.angularPosition) * 50)
        # sign =1
        #
        # if self.angularPosition < -math.pi + 0.5 or self.angularPosition > math.pi * 0.5:
        #     sign = -1
        # yMove=int(math.cos(self.angularPosition)*50) *self.sign
        yMove = int(math.cos(self.angularPosition) * 50)

        painter.drawLine(x1, y1, x1 - xMove, y1 - yMove)

        painter.setPen(QPen(Qt.gray, 5, Qt.DotLine))

        x1Bottom = self.angleWidget.x() + round(width * 0.2)
        x2Bottom = self.angleWidget.x() + round(width * 0.8)
        painter.drawLine(x1Bottom, y1, x2Bottom, y1)

        # painter.end()

    def angularPoseCallback(self, event):
        # print(event)
        self.angularPosition = event.angular_position
        self.angleWidget.update()
        # self.angleW

    def imuCallback(self, event):
        self.sign = 1
        if event.linear_acceleration.z < 0:
            self.sign = -1

        # print(self.sign)

        # self.angleWidget.update()

    @staticmethod
    def onDestroyed():
        # Do stuff here
        print("CCCCCCCCCCCCLOOOOOOOOOOOSEEEEEEEEEEEEEE333333333")
        # self.publisher= None

        pass

    def closeEvent(self, event):
        print("CCCCCCCCCCCCLOOOOOOOOOOOSEEEEEEEEEEEEEE")

    def __del__(self):
        print('Destructor called, vehicle deleted.')

    def shutdown_plugin(self):
        print("shutdown_plugin")


# self.batteryStatusSubscriber = self.node.create_subscription(BatteryStatus, self.namespace + '/internal/battery_status',
#                                                         self.batteryCallback, 10)
#
#
#       self.distance0Subscriber  = self.node.create_subscription(Range, self.namespace + '/internal/distance_0',
#                                                 self.frontSensorCallback, 10)
#
#       self.distance1Subscriber = self.node.create_subscription(Range, self.namespace + '/internal/distance_1', self.backSensorCallback, 10)
#
#       self.temperatureCpuSubscriber = self.node.create_subscription(Float32, self.namespace + '/internal/temperature_cpu',
#                                     self.temperatureCpuCallback, 10)
#
#       self.temperatureMainSubscriber = self.node.create_subscription(Float32, self.namespace + '/internal/temperature_main',
#                                     self.temperatureMainCallback, 10)
#
#       self.node.create_subscription(AngularPose, self.namespace + '/internal/angular_pose', self.angularPoseCallback,
#                                     10)
#       self.node.create_subscription(AngularPose, self.namespace + '/internal/imu', self.imuCallback, 10)

SubscriberParam = namedtuple('SubscriberParam', ["subscriber", "messageType", "topic", "callback"])
