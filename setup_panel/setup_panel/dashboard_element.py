# This Python file uses the following encoding: utf-8
import os

from python_qt_binding.QtWidgets import QPushButton, QWidget,QLineEdit,QTableWidget,QTableWidgetItem,QMessageBox
from ament_index_python import get_resource
from python_qt_binding import loadUi

import json

class DashboardElementWidget(QWidget):
    def __init__(self, node, plugin=None, fileName=None):
        super(DashboardElementWidget, self).__init__()

        self.fileName = fileName

        _, package_path = get_resource('packages', 'setup_panel')
        ui_file = os.path.join(package_path, 'share', 'setup_panel', 'resource', 'dashboard_element.ui')
        loadUi(ui_file, self)

        self.dataFilePath = os.path.join(package_path, 'share', 'setup_panel', 'data', 'robots',fileName)
        self.loadJson()

        self.modifyButton.clicked.connect(self.modifyButtonClicked)
        self.deleteButton.clicked.connect(self.deleteClicked)



    def loadJson(self):
        dataFile = open(self.dataFilePath)
        data = json.load(dataFile)
        dataFile.close()

        self.robotName=data['robotName']
        self.robotNameInput.setText(data['robotName'])


    def deleteClicked(self):
        print('deleteClicked')

        reply = QMessageBox.question(self, self.robotName, 'Are you sure to delete robot?',
        QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

        print(reply)

        if reply == QMessageBox.Yes:
            print(self.parent())
            print(self.parent().parent())
            print(self.parent().parent().parent())
            print(self.parent().parent().parent().parent())
            # print(self.parent().parent().parent().mygroupbox)
            self.parent().parent().parent().parent().myForm.removeRow(self)

            # self.r


        # msg = QMessageBox(self)
        # msg.setIcon(QMessageBox.Information)
        #
        # msg.setText("This is a message box")
        # msg.setInformativeText("This is additional information")
        # msg.setWindowTitle("MessageBox demo")
        # msg.setDetailedText("The details are as follows:")
        # msg.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)

    def modifyButtonClicked(self):
        print('aaaaaaaaaaaaaaaaaaaaaa')
        print(self.parentWidget())
        print(self.parent())
        print(self.parent().parent())
        print(self.parent().parent().parent())
        print(self.parent().parent().parent().parent().parent().parent())
        self.parent().parent().parent().parent().parent().parent().goToSettings(self.fileName)

