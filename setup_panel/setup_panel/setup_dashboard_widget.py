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

        self._loadUi()
        self._setupDashboardElements()

        self.addNewRobotButton.clicked.connect(self.addNewRobot)

        innerCommunication.addRobotSignal.connect(self.addDashboardElement)
        innerCommunication.deleteRobotSignal.connect(self.onDeleteRobotSignal)

    def _setupDashboardElements(self):
        self.groupBox = QGroupBox()
        self.myForm = QFormLayout()

        _, sharedPath = get_resource('packages', 'shared')
        dataFilePath = os.path.join(sharedPath, 'share', 'shared', 'data', 'robots')

        self.elementDictionary = {}

        for fileName in (os.listdir(dataFilePath)):
            element = DashboardElementWidget(self, fileName=fileName, stack=self.stack)
            self.elementDictionary[fileName] = element
            self.myForm.addRow(element)

        self.groupBox.setLayout(self.myForm)

        self.scrollArea.setWidget(self.groupBox)

    def _loadUi(self):
        _, packagePath = get_resource('packages', 'setup_panel')
        uiFile = os.path.join(packagePath, 'share', 'setup_panel', 'resource', 'setup_dashboard.ui')
        loadUi(uiFile, self)


    def onDeleteRobotSignal(self, data):
        fileName = data['fileName']
        element = self.elementDictionary[fileName]
        self.myForm.removeRow(element)
        self.update()

    def addDashboardElement(self, data):
        fileName = data['fileName']
        element = DashboardElementWidget(self, fileName=fileName, stack=self.stack)
        self.elementDictionary[fileName] = element
        self.myForm.addRow(element)
        self.update()

    def addNewRobot(self):
        self.stack.goToSettings()
