# This Python file uses the following encoding: utf-8
import math
import os
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QPainter, QPen
from ament_index_python import get_resource
from python_qt_binding import loadUi
from shared.base_widget.base_widget import BaseWidget

from rclpy.node import Node
from minirys_msgs.msg import BatteryStatus
from sensor_msgs.msg import Range

from python_qt_binding.QtWidgets import QProgressBar

from std_msgs.msg import Float32, String


class DashboardWidget(BaseWidget):
    def __init__(self, stack=None, node=Node):
        super(DashboardWidget, self).__init__()
        BaseWidget.__init__(self, stack)

        self.loadUi()

        self.initializeRobotsOptions()

        self.node = node
        self.subscription = self.node.create_subscription(BatteryStatus, '/internal/battery_status',
                                                          self.batteryCallback, 10)

        self.node.create_subscription(Range, '/internal/distance_0', self.frontSensorCallback, 10)
        self.node.create_subscription(Range, '/internal/distance_1', self.backSensorCallback, 10)

        self.node.create_subscription(Float32, '/internal/temperature_cpu', self.temperatureCpuCallback, 10)
        self.node.create_subscription(Float32, '/internal/temperature_main', self.temperatureMainCallback, 10)

        # TODO pewnie do usuniecia
        self.voltageCell1ProgressBar.setRange(3500, 5500)
        self.voltageCell2ProgressBar.setRange(3500, 5500)
        self.voltageCell3ProgressBar.setRange(3500, 5500)

        # self.frontSensorProgressBar.setRange(0, 4000)

        self.batteryUnderVoltageFlag = False

        self.angleWidget.paintEvent = self.paintRobotAngle
        # self.paintRobotAngle('a')
        # self.paintRobotAngle('b')
        # self.paintRobotAngle('c')

    # def paintEvent(self, event):
    #     # self.paintRobotAngle(event)
    #     # self.angleWidget.update()
    #     pass

    # def paintRobotAngle(self, event):
    #     painter = QPainter(self.angleWidget)
    #     # painter.setRenderHint(QPainter.Antialiasing)
    #     painter.setPen(Qt.red)
    #     painter.setBrush(Qt.white)
    #     painter.drawLine(50, 50, 100, 100)
    #     painter.end()

    def loadUi(self):
        _, packagePath = get_resource('packages', 'dashboard')
        uiFile = os.path.join(packagePath, 'share', 'dashboard', 'resource', 'dashboard.ui')
        loadUi(uiFile, self)

    def initializeSettings(self):
        print('initializeSettings')

    def batteryCallback(self, event):
        # print('event')
        # print(event)
        # # print( event.voltage_cell3)

        voltageCell1 = float('{:.3f}'.format(event.voltage_cell1))
        voltageCell2 = float('{:.3f}'.format(event.voltage_cell2))
        voltageCell3 = float('{:.3f}'.format(event.voltage_cell3))

        undervoltageWarning = event.undervoltage_warning
        undervoltageError = event.undervoltage_error

        voltageCell1P = int(voltageCell1 * 1000)
        voltageCell2P = int(voltageCell2 * 1000)
        voltageCell3P = int(voltageCell3 * 1000)

        # print(voltageCell1P)

        # self.voltageCell1ProgressBar.setValue(voltageCell1P)
        # self.voltageCell2ProgressBar.setValue(voltageCell2P)
        # self.voltageCell2ProgressBar.setValue(voltageCell3P)

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

        # self.temperatureMainLcd.setStyleSheet('color: black;')

    def frontSensorCallback(self, event):
        print(event.range)
        range = int((event.range * 1000))
        self.frontSensorProgressBar.setValue(range)
        self.frontDistanceLcd.display(event.range)

    def backSensorCallback(self, event):
        # print(event)
        range = int((event.range * 1000))
        # self.backSensorProgressBar.setValue(range)
        self.backDistanceLcd.display(event.range)
        # self.frontDistanceLcd.display(event.range)

    def paintRobotAngle(self, event):
        painter = QPainter()
        painter.begin(self.angleWidget)
        pos = self.angleWidget.pos()
        painter.setPen(QPen(Qt.black, 5, Qt.SolidLine))
        # painter.setBrush(QBrush(Qt.red, Qt.SolidPattern))
        print("pos")
        print(pos)
        print( self.angleWidget.mapToGlobal(pos))
        print( self.angleWidget.geometry())
        print(pos.x())
        pos1= self.angleWidget.mapToGlobal(pos)
        width=self.angleWidget.width()
        height=self.angleWidget.height()
        x1=self.angleWidget.x()+round(width*0.5)
        y1=self.angleWidget.y()+round(height*0.9)
        painter.drawLine(x1,y1 , x1,y1-50)


        painter.setPen(QPen(Qt.gray, 5, Qt.DotLine))

        x1Bottom=self.angleWidget.x()+round(width*0.2)
        x2Bottom=self.angleWidget.x()+round(width*0.8)
        painter.drawLine(x1Bottom,y1,x2Bottom,y1)

        # painter.end()
