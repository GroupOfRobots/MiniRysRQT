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
        self.currentDataFile = None
        self.data = None
        self.namespace = ''

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
            dataFromFile = json.load(dataFile)
            dataFile.close()

            self.addItemData(dataFromFile, filePath)

    def onUpdateRobotSignal(self, event):
        index = self.comboBox.findData(event)

        self.loadData(event)

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

    def onAddRobotSignal(self, signalData):
        self.loadData(signalData)
        self.addItemData( self.data, signaldData['filePath'])

    def setRobotOnScreen(self):
        currentData = self.comboBox.currentData()
        if currentData == None:
            return

        self.loadData(currentData)
        self.initializeRobotSettings()

    def loadData(self, currentData):
        self.dataFilePath = currentData['filePath']
        dataFile = open(self.dataFilePath)
        self.data = json.load(dataFile)
        dataFile.close()

        self.namespace = self.data.get('namespace', '')


    def addItemData(self, data, filePath):
        id = data.get('id', None)

        robotName = data.get('robotName', '')

        itemData = {
            "fileName": None,
            "filePath": filePath,
            "id": id,
        }

        self.comboBox.addItem(robotName, itemData)

    @abstractmethod
    def initializeRobotSettings(self):
        print('abstractmethod')
        pass
