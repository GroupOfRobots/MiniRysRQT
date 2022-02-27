# This Python file uses the following encoding: utf-8
import os
import os.path

from ament_index_python import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QGroupBox, QFormLayout
from .dashboard_element import DashboardElementWidget
from shared.inner_communication import innerCommunication

class SetupDashboardWidget(QWidget):
    def __init__(self, stack=None):
        super(SetupDashboardWidget, self).__init__()

        self.stack = stack

        self.loadUi()

        _, sharedPath = get_resource('packages', 'shared')
        self.dataFilePath = os.path.join(sharedPath, 'share', 'shared', 'data', 'robots')

        self.elementDictionary = {}

        self.groupBox = QGroupBox()
        self.form = QFormLayout()
        self.setupDashboardElements()

        self.addNewRobotButton.clicked.connect(self.addNewRobot)

        innerCommunication.addRobotSignal.connect(self.addDashboardElement)
        innerCommunication.deleteRobotSignal.connect(self.onDeleteRobotSignal)
        innerCommunication.updateRobotSignal.connect(self.onUpdateRobotSignal)

    def setupDashboardElements(self):
        for fileName in (os.listdir(self.dataFilePath)):
            element = DashboardElementWidget(fileName=fileName, stack=self.stack)
            self.elementDictionary[self.dataFilePath + '/'+ fileName] = element
            self.form.addRow(element)

        self.groupBox.setLayout(self.form)

        self.scrollArea.setWidget(self.groupBox)

    def loadUi(self):
        _, packagePath = get_resource('packages', 'setup_panel')
        uiFile = os.path.join(packagePath, 'share', 'setup_panel', 'resource', 'setup_dashboard.ui')
        loadUi(uiFile, self)

    def onDeleteRobotSignal(self, data):
        dataFilePath = data['filePath']
        element = self.elementDictionary[dataFilePath]
        self.form.removeRow(element)
        self.update()

    def addDashboardElement(self, data):
        fileName = data['filePath']
        element = DashboardElementWidget(self, fileName=fileName, stack=self.stack)
        self.elementDictionary[self.dataFilePath + '/'+ fileName] = element
        self.form.addRow(element)
        self.update()

    def onUpdateRobotSignal(self, data):
        filePath = data['filePath']
        element = self.elementDictionary[filePath]
        element.loadJson()

    def addNewRobot(self):
        self.stack.goToSettings()
