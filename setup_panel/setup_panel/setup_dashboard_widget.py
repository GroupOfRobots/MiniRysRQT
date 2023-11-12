# This Python file uses the following encoding: utf-8
import os
import os.path

from ament_index_python import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from shared.inner_communication import innerCommunication

from .dashboard_element import DashboardElementWidget


class SetupDashboardWidget(QWidget):
    def __init__(self, stack=None, node=None, mainPanel=None):
        super(SetupDashboardWidget, self).__init__()

        self.stack = stack
        self.node=node

        self.loadUi()

        _, sharedPath = get_resource('packages', 'shared')
        self.dataFilePath = os.path.join(sharedPath, 'share', 'shared', 'data', 'robots')

        self.elementDictionary = {}

        self.setupDashboardElements()

        self.addNewRobotButton.clicked.connect(self.addNewRobot)

        innerCommunication.addRobotSignal.connect(self.addDashboardElement)
        innerCommunication.deleteRobotSignal.connect(self.onDeleteRobotSignal)
        innerCommunication.updateRobotSignal.connect(self.onUpdateRobotSignal)

    def setupDashboardElements(self):
        for fileName in (os.listdir(self.dataFilePath)):
            element = DashboardElementWidget(fileName=fileName, stack=self.stack)
            self.elementDictionary[self.dataFilePath + '/' + fileName] = element
            # self.elements.addWidget(element)
            self.elementsBoxLayoutUI.addWidget(element)

    def loadUi(self):
        _, packagePath = get_resource('packages', 'setup_panel')
        uiFile = os.path.join(packagePath, 'share', 'setup_panel', 'resource', 'setup_dashboard.ui')
        loadUi(uiFile, self)

    def onDeleteRobotSignal(self, data):
        dataFilePath = data['filePath']
        element = self.elementDictionary[dataFilePath]
        self.elementDictionary.pop(dataFilePath)
        self.elementsBoxLayoutUI.removeWidget(element)
        self.elementsBoxLayoutUI.update()


    def addDashboardElement(self, data):
        fileName = data['fileName']
        element = DashboardElementWidget(fileName=fileName, stack=self.stack)
        self.elementDictionary[self.dataFilePath + '/' + fileName] = element
        self.elementsBoxLayoutUI.addWidget(element)
        self.elementsBoxLayoutUI.update()

    def onUpdateRobotSignal(self, data):
        filePath = data['filePath']
        element = self.elementDictionary[filePath]
        element.loadJson()

    def addNewRobot(self):
        self.stack.goToSettings(addMode=True)
