# This Python file uses the following encoding: utf-8
import os

from python_qt_binding.QtWidgets import QWidget, QStackedWidget, QStackedLayout
from ..deleted_robot_screen.deleted_robot_screen_widget import DeletedRobotScreenWidget
from setup_panel.details.setup_widget import SetupWidget

from ament_index_python import get_resource
from shared.no_robot_configuration_screen.no_robot_configuration_screen_widget import NoRobotConfigurationScreenWidget
from abc import abstractmethod

class StackWidget(QWidget):
    def __init__(self, node=None,constructor=None, mainPanel=None):
        super(StackWidget, self).__init__()
        self.stack = QStackedWidget(self)
        self.node=node
        self.constructor=constructor
        self.mainPanel=mainPanel

        layout = QStackedLayout(self)
        layout.addWidget(self.stack)

        self.setLayout(layout)

        self.mainChildWidget = None
        self.initWidget()

    @abstractmethod
    def initWidget(self):
        if self.checkIfRobotsConfigurationFilesExists():
            self.createWidget()
        else:
            self.noRobotConfigurationScreenWidget=NoRobotConfigurationScreenWidget(stack = self, node= self.node)
            self.mainChildWidget =self.noRobotConfigurationScreenWidget
            self.stack.addWidget(self.mainChildWidget)

    def addFirstRobot(self):
        self.stack.removeWidget(self.noRobotConfigurationScreenWidget)
        self.createWidget()

    def createWidget(self):
        if self.mainPanel is None:
            self.mainChildWidget = self.constructor(stack=self, node=self.node)
        else:
            self.mainChildWidget = self.constructor(stack=self, node=self.node, mainPanel=self.mainPanel)
        self.stack.addWidget(self.mainChildWidget)

    def checkIfRobotsConfigurationFilesExists(self):
        dataFilePath = self.getDataFilePath()
        filesDirs = os.listdir(dataFilePath)
        return len(filesDirs) > 0

    def getDataFilePath(self):
        _, shared_package_path = get_resource('packages', 'shared')
        return os.path.join(shared_package_path, 'share', 'shared', 'data', 'robots')

    def goToDeletedRobotScreen(self):
        self.deletedRobotScreenWidget = DeletedRobotScreenWidget(stack=self)
        self.stack.addWidget(self.deletedRobotScreenWidget)
        self.stack.setCurrentWidget(self.deletedRobotScreenWidget)

    def onDeletedRobotScreenReturn(self):
        self.mainChildWidget.setRobotOnScreen()
        self.stack.setCurrentWidget(self.mainChildWidget)
        self.stack.removeWidget(self.deletedRobotScreenWidget)
        self.deletedRobotScreenWidget.deleteLater()
        self.deletedRobotScreenWidget = None

    def goToSettings(self, dataFilePath=None):
        if hasattr(self, 'setupWidget'):
            self.stack.removeWidget(self.setupWidget)
        self.setupWidget = SetupWidget(stack=self, dataFilePath=dataFilePath, node=self.node)
        self.stack.addWidget(self.setupWidget)
        self.stack.setCurrentWidget(self.setupWidget)
