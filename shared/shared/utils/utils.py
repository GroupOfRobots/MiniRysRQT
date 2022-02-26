import json
import os

from ament_index_python import get_resource
from shared.inner_communication import innerCommunication


def initializeRobotsOptions(comboBox):
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
            "fileName": fileName,
            "filePath": filePath,
            "id": id,
        }

        comboBox.addItem(robotName, itemData)

class OptionsManager:
    def __init__(self, comboBox = None,stack = None):
        self.comboBox = comboBox
        self.stack = stack
        print("OptionsManager")
        print(stack)
        innerCommunication.deleteRobotSignal.connect(self.onDeleteRobotSignal)

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
                "fileName": fileName,
                "filePath": filePath,
                "id": id,
            }

            self.comboBox.addItem(robotName, itemData)

    def onDeleteRobotSignal(self, data):
        indexOfElementToBeRemoved = self.comboBox.findData(data)

        if indexOfElementToBeRemoved == self.comboBox.currentIndex():
            self.stack.goToDeletedRobotScreen()

        self.comboBox.removeItem(indexOfElementToBeRemoved)