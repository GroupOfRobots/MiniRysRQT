# This Python file uses the following encoding: utf-8

from shared.inner_communication import innerCommunication
from python_qt_binding.QtWidgets import QPushButton, QWidget,QLineEdit,QTableWidget,QTableWidgetItem,QMessageBox
from ament_index_python import get_resource
from python_qt_binding import loadUi

import json
import os

class DashboardElementWidget(QWidget):
    def __init__(self, fileName=None, stack=None):
        super(DashboardElementWidget, self).__init__()

        self.fileName = fileName
        self.stack=stack

        self.loadUi()
        _, shared_path = get_resource('packages', 'shared')

        self.dataFilePath = os.path.join(shared_path, 'share', 'shared', 'data', 'robots',fileName)
        self.loadJson()

        self.modifyButton.clicked.connect(self.modifyButtonClicked)
        self.deleteButton.clicked.connect(self.deleteClicked)

    def loadUi(self):
        _, packagePath = get_resource('packages', 'setup_panel')
        uiFile = os.path.join(packagePath, 'share', 'setup_panel', 'resource', 'dashboard_element.ui')
        loadUi(uiFile, self)

    def loadJson(self):
        dataFile = open(self.dataFilePath)
        data = json.load(dataFile)
        dataFile.close()

        self.robotName=data['robotName']
        self.id=data['id']
        self.robotNameInput.setText(data['robotName'])


    def deleteClicked(self):

        reply = QMessageBox.question(self, self.robotName, 'Are you sure to delete robot?',
        QMessageBox.Yes | QMessageBox.No, QMessageBox.No)


        if reply == QMessageBox.Yes:
            os.remove(self.dataFilePath)

        itemData = {
            "fileName": None,
            "filePath": self.dataFilePath,
            "id": self.id,
        }

        innerCommunication.deleteRobotSignal.emit(itemData)

    def modifyButtonClicked(self):
        self.stack.goToSettings(self.dataFilePath)
