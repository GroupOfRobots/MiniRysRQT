# This Python file uses the following encoding: utf-8

import json
import os

from ament_index_python import get_resource
from python_qt_binding.QtWidgets import QMessageBox
from python_qt_binding.QtWidgets import QWidget
from shared.enums import PackageNameEnum
from shared.inner_communication import innerCommunication
from shared.utils.load_ui_file import loadUiFile


class DashboardElementWidget(QWidget):
    def __init__(self, fileName=None, stack=None):
        super(DashboardElementWidget, self).__init__()

        self.fileName = fileName
        self.stack = stack

        loadUiFile(self, PackageNameEnum.SetupPanel, 'dashboard_element.ui')
        _, shared_path = get_resource('packages', 'shared')

        self.dataFilePath = os.path.join(shared_path, 'share', 'shared', 'data', 'robots', fileName)
        self.loadJson()

        self.modifyButton.clicked.connect(self.modifyButtonClicked)
        self.deleteButton.clicked.connect(self.deleteClicked)

    def loadJson(self):
        dataFile = open(self.dataFilePath)
        data = json.load(dataFile)
        dataFile.close()

        self.robotName = data.get('robotName')
        self.id = data.get('id')
        self.robotNameInput.setText(data.get('robotName'))

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
