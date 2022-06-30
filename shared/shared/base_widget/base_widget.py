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
        self.data = None

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
            self.data = json.load(dataFile)
            dataFile.close()
            robotName = self.data.get('robotName','')
            id = self.data.get('id', None)

            itemData = {
                "fileName": None,
                "filePath": filePath,
                "id": id,
            }

            self.comboBox.addItem(robotName, itemData)

    def onUpdateRobotSignal(self, event):
        index = self.comboBox.findData(event)
        filePath = event['filePath']
        dataFile = open(filePath)
        self.data = json.load(dataFile)
        dataFile.close()
        robotName = self.data.get('robotName','')
        self.comboBox.setItemText(index, robotName)

        if index == self.comboBox.currentIndex():
            # DO sprawdzenia
            self.setRobotOnScreen()

        self.update()

    def onDeleteRobotSignal(self, data):
        indexOfElementToBeRemoved = self.comboBox.findData(data)

        if indexOfElementToBeRemoved == self.comboBox.currentIndex():
            self.stack.goToDeletedRobotScreen()

        self.comboBox.removeItem(indexOfElementToBeRemoved)

    def onAddRobotSignal(self, signaldData):
        fileName = signaldData['fileName']
        filePath = signaldData['filePath']
        id = signaldData['id']

        dataFile = open(filePath)
        self.data = json.load(dataFile)
        dataFile.close()
        robotName = self.data.get('robotName','')

        itemData = {
            "fileName": None,
            "filePath": filePath,
            "id": id,
        }

        self.comboBox.addItem(robotName, itemData)

    def setRobotOnScreen(self):
        currentData = self.comboBox.currentData()
        if currentData == None:
            return

        filePath = currentData['filePath']
        dataFile = open(filePath)
        self.data = json.load(dataFile)
        dataFile.close()
        self.initializeRobotSettings()

    @abstractmethod
    def initializeRobotSettings(self):
        print('abstractmethod')
        pass
