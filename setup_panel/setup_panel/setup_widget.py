# This Python file uses the following encoding: utf-8
import os

# from PyQt5.QtWidgets import QMessageBox
from python_qt_binding.QtWidgets import QPushButton, QWidget, QLineEdit, QTableWidget, QTableWidgetItem, QMessageBox
from ament_index_python import get_resource
from python_qt_binding import loadUi

import json

from shared.enums import ControlKeyEnum
from shared.inner_communication import innerCommunication

# from setup_panel.dashboard_element import DashboardElementWidget


import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from .dashboard_element import DashboardElementWidget \

class SetupWidget(QWidget):
    def __init__(self, node=None, plugin=None, fileName=None, context=None):
        super(SetupWidget, self).__init__()

        self.node = node

        self.stack = plugin

        _, self.shared_path = get_resource('packages', 'shared')
        _, self.package_path = get_resource('packages', 'setup_panel')
        ui_file = os.path.join(self.package_path, 'share', 'setup_panel', 'resource', 'setup.ui')
        loadUi(ui_file, self)

        self.defaultFilePath = os.path.join(self.shared_path, 'share', 'shared', 'data', 'default.json')

        self.addMode = False

        self.fileName = fileName

        if fileName:
            self.dataFilePath = os.path.join(self.shared_path, 'share', 'shared', 'data', 'robots', fileName)
            self.data = self.loadData(self.dataFilePath)
        else:
            self.addMode = True
            self.dataFilePath = os.path.join(self.shared_path, 'share', 'shared', 'data', 'robots')
            self.data = self.loadData(self.defaultFilePath)

            currentFiles = os.listdir(self.dataFilePath)
            for index in range(len(os.listdir(self.dataFilePath)) + 1):
                self.fileName = 'data' + str(index) + '.json'
                if self.fileName in currentFiles:
                    continue
                else:
                    break
            self.dataFilePath += '/' + self.fileName

        self.loadJson()

        self.backButton.clicked.connect(self.goBack)
        self.saveButton.clicked.connect(self.saveClicked)

        self.restoreDefaultButton.clicked.connect(self.restoreDefault)

        self.keyInputDictionary = {
            ControlKeyEnum.FORWARD: self.forwardKeyInput,
            ControlKeyEnum.RIGHT: self.rightKeyInput,
            ControlKeyEnum.BACKWARD: self.backwardKeyInput,
            ControlKeyEnum.LEFT: self.leftKeyInput,
        }

        self.addControlKeysValidators()

    def addControlKeysValidators(self):
        self.keyInputDictionary[ControlKeyEnum.FORWARD].textChanged.connect(
            lambda event: self.validator(event, ControlKeyEnum.FORWARD))
        self.keyInputDictionary[ControlKeyEnum.RIGHT].textChanged.connect(
            lambda event: self.validator(event, ControlKeyEnum.RIGHT))
        self.keyInputDictionary[ControlKeyEnum.BACKWARD].textChanged.connect(
            lambda event: self.validator(event, ControlKeyEnum.BACKWARD))
        self.keyInputDictionary[ControlKeyEnum.LEFT].textChanged.connect(
            lambda event: self.validator(event, ControlKeyEnum.LEFT))

    def validator(self, event, key):
        button = self.keyInputDictionary[key]
        newValue = event.upper()
        button.setText(newValue)
        self.controlKeys[key] = newValue

        valuesMap = {}

        for key, value in self.controlKeys.items():
            if value in valuesMap:
                valuesMap[value].append(key)
            else:
                valuesMap[value] = [key]

        disableSaveButton = False
        for _, buttonKeys in valuesMap.items():
            if len(buttonKeys) > 1:
                for inputKey in buttonKeys:
                    color = '#EB5535'  # red
                    self.keyInputDictionary[inputKey].setStyleSheet('QLineEdit { background-color: %s }' % color)
                    disableSaveButton = True
            else:
                self.keyInputDictionary[buttonKeys[0]].setStyleSheet('')

        if disableSaveButton:
            self.saveButton.setEnabled(False)
        else:
            self.saveButton.setEnabled(True)

    def restoreDefault(self):
        reply = QMessageBox.question(self, 'Restore default settings', 'Are you sure to restore default setting?',
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

        if reply == QMessageBox.Yes:
            self.data = self.loadData(self.defaultFilePath)
            self.loadJson()

    def __del__(self):
        print('eeeeeeeeeeennnnnnnnnnnndddddddd')
        # rclpy.shutdown()

    def goBack(self):
        self.stack.stack.setCurrentIndex(0)

    def saveClicked(self):
        if (self.addMode):
            self.addNewRobot()
            return
        self.updateRobotData()

    def addNewRobot(self):
        self.data['robotName'] = self.robotNameInput.text()
        self.data['controlKeys']['forward'] = self.forwardKeyInput.text()

        # save to new file
        dataFile = open(self.dataFilePath, 'x')
        json.dump(self.data, dataFile)
        dataFile.close()

        parent = self.parent()
        setupDashboard = parent.widget(0)
        dashboardElement = DashboardElementWidget(self, fileName=self.fileName)
        setupDashboard.myForm.addRow(dashboardElement)

        innerCommunication.addRobotSignal.emit()
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

        innerCommunication.closeApp.emit()
        self.goBack()

    def test1(self):
        print('qqqqqqqqqqqqqqqwwwwwwwwwwwwwwwwwww aaaaaaaaaaaaa')

    def resizeEvent(self, event):
        pass
        # print("resize")

    def loadData(self, filePath):
        dataFile = open(filePath)
        data = json.load(dataFile)
        dataFile.close()
        return data

    def loadJson(self):
        self.robotNameInput.setText(self.data['robotName'])

        # CONTROL KEYS
        self.controlKeys = self.data['controlKeys']
        self.forwardKeyInput.setText(self.controlKeys[ControlKeyEnum.FORWARD])
        self.rightKeyInput.setText(self.controlKeys[ControlKeyEnum.RIGHT])
        self.backwardKeyInput.setText(self.controlKeys[ControlKeyEnum.BACKWARD])
        self.leftKeyInput.setText(self.controlKeys[ControlKeyEnum.LEFT])

        # DYNAMIC
        dynamic = self.data['dynamic']
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
        joystick = self.data['joystick']

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