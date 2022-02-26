# This Python file uses the following encoding: utf-8
import os
import threading
import time

from ament_index_python import get_resource
from python_qt_binding import QtCore
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QPoint
from python_qt_binding.QtWidgets import QWidget
from shared.enums import ControlKeyEnum

from shared.inner_communication import innerCommunication

import json

class BaseWidget(QWidget):
    def __init__(self, node=None, plugin=None,stack=None):
        super(BaseWidget, self).__init__()

        self.stack=stack

        self.setFocusPolicy(Qt.ClickFocus)
        self.setFocus()

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