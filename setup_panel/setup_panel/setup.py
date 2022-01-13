# This Python file uses the following encoding: utf-8
import os

# from PyQt5.QtWidgets import QMessageBox
from python_qt_binding.QtWidgets import QPushButton, QWidget, QLineEdit, QTableWidget, QTableWidgetItem
from ament_index_python import get_resource
from python_qt_binding import loadUi

import json

from setup_panel.dashboard_element import DashboardElementWidget


import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class SetupWidget(QWidget):
    def __init__(self, node, plugin=None, fileName=None):
        super(SetupWidget, self).__init__()

        self.node = node

        _, self.package_path = get_resource('packages', 'setup_panel')
        ui_file =os.path.join(self.package_path, 'share', 'setup_panel', 'resource', 'setup.ui')
        loadUi(ui_file, self)

        self.defaultFilePath = os.path.join(self.package_path, 'share', 'setup_panel', 'data', 'default.json')

        self.addMode = False

        if fileName:
            self.dataFilePath = os.path.join(self.package_path, 'share', 'setup_panel', 'data', 'robots', fileName)
            data = self.loadData(self.dataFilePath)
        else:
            self.addMode = True
            self.dataFilePath = os.path.join(self.package_path, 'share', 'setup_panel', 'data', 'robots')
            data = self.loadData(self.defaultFilePath)

            currentFiles = os.listdir(self.dataFilePath)
            for index in range(len(os.listdir(self.dataFilePath)) + 1):
                self.fileName = 'data' + str(index) + '.json'
                if self.fileName in currentFiles:
                    continue
                else:
                    break
            self.dataFilePath+='/'+self.fileName

        self.loadJson(data)

        self.backButton.clicked.connect(self.backClicked)
        self.saveButton.clicked.connect(self.saveClicked)

        print('wwwwwwwwwwwws')
        print(self.node)
        print(        self.node)


        # self._context.node

        self.publisher_ = self.node.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.node.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def __del__(self):
        print('eeeeeeeeeeennnnnnnnnnnndddddddd')
        # rclpy.shutdown()


    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        msg.data += self.fileName
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def goBack(self):
        parent = self.parent()
        parent.setCurrentIndex(0)

    def backClicked(self):
        self.goBack()

    def saveClicked(self):
        if(self.addMode):
            self.addNewRobot()
            return
        self.updateRobotData()

    def addNewRobot(self):
        defaultFile = open(self.defaultFilePath, 'r')
        data = json.load(defaultFile)
        defaultFile.close()
        data['robotName'] = self.robotNameInput.text()
        data['controlKeys']['forward'] = self.forwardKeyInput.text()

        dataFile = open(self.dataFilePath, 'x')
        json.dump(data, dataFile)
        dataFile.close()

        parent = self.parent()
        setupDashboard = parent.widget(0)
        dashboardElement= DashboardElementWidget(self,fileName=self.fileName)
        setupDashboard.myForm.addRow(dashboardElement)

        self.goBack()


    def updateRobotData(self):
        dataFile = open(self.dataFilePath, 'r')
        data = json.load(dataFile)
        dataFile.close()
        dataFile = open(self.dataFilePath, 'w')
        data['robotName'] = self.robotNameInput.text()
        data['controlKeys']['forward'] = self.forwardKeyInput.text()
        json.dump(data, dataFile)
        dataFile.close()
        self.goBack()

    def resizeEvent(self, event):
        print("resize")

    def loadData(self, filePath):
        dataFile = open(filePath)
        data = json.load(dataFile)
        dataFile.close()
        return data

    def loadJson(self, data):

        self.robotNameInput.setText(data['robotName'])

        # CONTROL KEYS
        controlKeys = data['controlKeys']
        self.forwardKeyInput.setText(controlKeys['forward'])
        self.rightKeyInput.setText(controlKeys['right'])
        self.backwardKeyInput.setText(controlKeys['backward'])
        self.leftKeyInput.setText(controlKeys['left'])

        # DYNAMIC
        dynamic = data['dynamic']
        forwardDynamic = dynamic['forward']
        forwardLeftDynamic = dynamic['forwardLeft']
        forwardRightDynamic = dynamic['forwardRight']

        backwardDynamic = dynamic['backward']
        backwardLeftDynamic = dynamic['backward']
        backwardRightDynamic = dynamic['backward']

        # FORWARD
        self.dynamicTable.setItem(0, 0, QTableWidgetItem(str(forwardDynamic['leftEngine'])))
        self.dynamicTable.setItem(0, 1, QTableWidgetItem(str(forwardDynamic['rightEngine'])))
        self.dynamicTable.setItem(0, 2, QTableWidgetItem(str(forwardDynamic['inertia'])))

        self.dynamicTable.setItem(1, 0, QTableWidgetItem(str(forwardLeftDynamic['leftEngine'])))
        self.dynamicTable.setItem(1, 1, QTableWidgetItem(str(forwardLeftDynamic['rightEngine'])))
        self.dynamicTable.setItem(1, 2, QTableWidgetItem(str(forwardLeftDynamic['inertia'])))

        self.dynamicTable.setItem(2, 0, QTableWidgetItem(str(forwardRightDynamic['leftEngine'])))
        self.dynamicTable.setItem(2, 1, QTableWidgetItem(str(forwardRightDynamic['rightEngine'])))
        self.dynamicTable.setItem(2, 2, QTableWidgetItem(str(forwardRightDynamic['inertia'])))

        # BACKWARD
        self.dynamicTable.setItem(3, 0, QTableWidgetItem(str(backwardDynamic['leftEngine'])))
        self.dynamicTable.setItem(3, 1, QTableWidgetItem(str(backwardDynamic['rightEngine'])))
        self.dynamicTable.setItem(3, 2, QTableWidgetItem(str(backwardDynamic['inertia'])))

        self.dynamicTable.setItem(4, 0, QTableWidgetItem(str(backwardLeftDynamic['leftEngine'])))
        self.dynamicTable.setItem(4, 1, QTableWidgetItem(str(backwardLeftDynamic['rightEngine'])))
        self.dynamicTable.setItem(4, 2, QTableWidgetItem(str(backwardLeftDynamic['inertia'])))

        self.dynamicTable.setItem(5, 0, QTableWidgetItem(str(backwardRightDynamic['leftEngine'])))
        self.dynamicTable.setItem(5, 1, QTableWidgetItem(str(backwardRightDynamic['rightEngine'])))
        self.dynamicTable.setItem(5, 2, QTableWidgetItem(str(backwardRightDynamic['inertia'])))

        # JOYSTICK
        joystick = data['joystick']

        joystickForward = joystick['forward']
        joystickRight = joystick['right']
        joystickBackward = joystick['backward']
        joystickLeft = joystick['left']

        self.joystickForward.setItem(0, 0, QTableWidgetItem(str(joystickForward['leftEngine'])))
        self.joystickForward.setItem(0, 1, QTableWidgetItem(str(joystickForward['rightEngine'])))

        self.joystickRight.setItem(0, 0, QTableWidgetItem(str(joystickRight['leftEngine'])))
        self.joystickRight.setItem(0, 1, QTableWidgetItem(str(joystickRight['rightEngine'])))

        self.joystickBackward.setItem(0, 0, QTableWidgetItem(str(joystickBackward['leftEngine'])))
        self.joystickBackward.setItem(0, 1, QTableWidgetItem(str(joystickBackward['rightEngine'])))

        self.joystickLeft.setItem(0, 0, QTableWidgetItem(str(joystickLeft['leftEngine'])))
        self.joystickLeft.setItem(0, 1, QTableWidgetItem(str(joystickLeft['rightEngine'])))
