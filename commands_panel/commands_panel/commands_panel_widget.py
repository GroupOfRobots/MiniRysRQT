# This Python file uses the following encoding: utf-8
import json
import os

# from PyQt5.QtWidgets import QAbstractSpinBox
# from python_qt_binding.QtWidgets 
from python_qt_binding.QtWidgets import QAbstractSpinBox
from shared.base_widget.base_widget import BaseWidget

from std_msgs.msg import Float32
from ament_index_python import get_resource
from python_qt_binding import loadUi

from .command_execute_element import CommandExecuteElementWidget


class CommandsPanelWidget(BaseWidget):
    def __init__(self, stack=None, node=None):
        super(CommandsPanelWidget, self).__init__()
        BaseWidget.__init__(self, stack)

        self.loadUI()
        self.initializeRobotsOptions()

        self.setupDashboardElements()

        self.comboBox.currentIndexChanged.connect(self.onChoosenRobotChange)
        currentData = self.comboBox.currentData()
        if currentData:
            self.dataFilePath = currentData['filePath']

            self.initializeSettings(self.dataFilePath)

    def onChoosenRobotChange(self, event):
        data = self.comboBox.currentData()
        if data:
            self.setRobotOnScreen(data)

    def initializeSettings(self, filePath):
        dataFile = open(filePath)
        data = json.load(dataFile)
        dataFile.close()
        # print(data)

    def setupDashboardElements(self):
        for i in range(1, 5):
            # print(self.www)
            element = CommandExecuteElementWidget()
            self.elements.addWidget(element)
            # self.groupBox.addWidget(element)

    def loadUI(self):
        _, packagePath = get_resource('packages', 'commands_panel')
        uiFile = os.path.join(packagePath, 'share', 'commands_panel', 'resource', 'commands_panel.ui')
        loadUi(uiFile, self)
