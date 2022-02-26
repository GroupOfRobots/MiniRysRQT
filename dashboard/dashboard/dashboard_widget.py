# This Python file uses the following encoding: utf-8
import os

from python_qt_binding.QtWidgets import QPushButton, QWidget,QLineEdit,QTableWidget,QTableWidgetItem,QMessageBox
from ament_index_python import get_resource
from python_qt_binding import loadUi

import json

from shared.utils.utils import initializeRobotsOptions
from shared.inner_communication import innerCommunication

class DashboardWidget(QWidget):
    def __init__(self, node, plugin=None, fileName=None, stack = None):
        super(DashboardWidget, self).__init__()

        self.fileName = fileName
        
        self.dashboardStack = stack

        _, package_path = get_resource('packages', 'dashboard')
        ui_file = os.path.join(package_path, 'share', 'dashboard', 'resource', 'dashboard.ui')
        loadUi(ui_file, self)

        print('wwwwwwwwwwwwww')

        initializeRobotsOptions(self.comboBox)

        # innerCommunication.addRobotSignal.connect(self.initializeRobotsOptions)
        innerCommunication.deleteRobotSignal.connect(self.onDeleteRobotSignal)

    def onDeleteRobotSignal(self, data):
        indexOfElementToBeRemoved = self.comboBox.findData(data)

        if indexOfElementToBeRemoved == self.comboBox.currentIndex():
            self.dashboardStack.goToDeletedRobotScreen()

        self.comboBox.removeItem(indexOfElementToBeRemoved)
