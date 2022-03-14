# This Python file uses the following encoding: utf-8
import json
import os
from abc import abstractmethod

from ament_index_python import get_resource
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import QWidget
from shared.inner_communication import innerCommunication


class BaseWidget(QWidget):
    def __init__(self, stack=None):
        super(BaseWidget, self).__init__()

        self.stack = stack

        self.setFocusPolicy(Qt.ClickFocus)
        self.setFocus()

        innerCommunication.deleteRobotSignal.connect(self.onDeleteRobotSignal)
        innerCommunication.addRobotSignal.connect(self.onAddRobotSignal)
        innerCommunication.updateRobotSignal.connect(self.onUpdateRobotSignal)

    def initializeRobotsOptions(self):
        _, shared_package_path = get_resource('packages', 'shared')
        dataFilePath = os.path.join(shared_package_path, 'share', 'shared', 'data', 'robots')

        for index, fileName in enumerate(os.listdir(dataFilePath)):
            filePath = dataFilePath + '/' + fileName
            dataFile = open(filePath)
            data = json.load(dataFile)
            dataFile.close()
            robotName = data['robotName']
            id = data['id']

            itemData = {
                "fileName": None,
                "filePath": filePath,
                "id": id,
            }

            self.comboBox.addItem(robotName, itemData)

    def onUpdateRobotSignal(self, data):
        index = self.comboBox.findData(data)
        filePath = data['filePath']
        dataFile = open(filePath)
        data = json.load(dataFile)
        dataFile.close()
        robotName = data['robotName']
        self.comboBox.setItemText(index, robotName)

        if index == self.comboBox.currentIndex():
            self.initializeSettings(filePath)

        self.update()

    def onDeleteRobotSignal(self, data):
        indexOfElementToBeRemoved = self.comboBox.findData(data)

        if indexOfElementToBeRemoved == self.comboBox.currentIndex():
            self.stack.goToDeletedRobotScreen()

        self.comboBox.removeItem(indexOfElementToBeRemoved)

    def onAddRobotSignal(self, data):
        fileName = data['fileName']
        filePath = data['filePath']
        id = data['id']

        dataFile = open(filePath)
        data = json.load(dataFile)
        dataFile.close()
        robotName = data['robotName']

        itemData = {
            "fileName": None,
            "filePath": filePath,
            "id": id,
        }

        self.comboBox.addItem(robotName, itemData)

    def setRobotOnScreen(self, data):
        filePath = data['filePath']
        index = self.comboBox.findData(data)
        self.comboBox.setCurrentIndex(index)
        self.initializeSettings(filePath)

    @abstractmethod
    def initializeSettings(self, filePath):
        print('abstractmethod')
        pass
