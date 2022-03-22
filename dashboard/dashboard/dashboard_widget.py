# This Python file uses the following encoding: utf-8
import os

from python_qt_binding.QtGui import QPainter, Qt
from ament_index_python import get_resource
from python_qt_binding import loadUi
from shared.base_widget.base_widget import BaseWidget

from rclpy.node import Node
from minirys_msgs.msg import BatteryStatus

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

        self.node.create_subscription(Float32, '/internal/temperature_cpu', self.temperatureCpuCallback, 10)
        self.node.create_subscription(Float32, '/internal/temperature_main', self.temperatureMainCallback, 10)

        # TODO pewnie do usuniecia
        self.voltageCell1ProgressBar.setRange(3500, 5500)
        self.voltageCell2ProgressBar.setRange(3500, 5500)
        self.voltageCell3ProgressBar.setRange(3500, 5500)

        self.batteryUnderVoltageFlag = False

        self.angleWidget.paintEvent =self.paintRobotAngle
        # self.paintRobotAngle('a')
        # self.paintRobotAngle('b')
        # self.paintRobotAngle('c')


    def paintEvent(self, event):
        # self.paintRobotAngle(event)
        # self.angleWidget.update()
        pass

    def paintRobotAngle(self, event):
        painter = QPainter(self.angleWidget)
        # painter.setRenderHint(QPainter.Antialiasing)
        painter.setPen(Qt.red)
        painter.setBrush(Qt.white)
        painter.drawLine(50, 50, 100, 100)

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
            self.batteryGroupBox.setStyleSheet(self.previousStyleSheet + 'QGroupBox { background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 red, stop: 1 pink);}')
            self.batteryGroupBox.setToolTip('<h2>ERROR</h2><div>Battery voltage is too low</div>')
            self.batteryGroupBox.setToolTipDuration(5000)
            self.batteryUnderVoltageFlag = True

        elif underVoltageWarning and not self.batteryUnderVoltageFlag:
            self.previousStyleSheet = self.batteryGroupBox.styleSheet()
            self.batteryGroupBox.setStyleSheet(self.previousStyleSheet + 'QGroupBox { background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 orange, stop: 1 yellow);}')
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
