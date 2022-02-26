# This Python file uses the following encoding: utf-8
from python_qt_binding.QtWidgets import QWidget, QStackedWidget,QHBoxLayout,QListWidget,QGridLayout,QLayout

from setup_panel.setup_widget import SetupWidget
from .control_panel_widget import ControlPanelWidget
from shared.deleted_robot_screen.deleted_robot_screen import DeletedRobotScreenWidget

class ControlPanelStack(QWidget):
    def __init__(self, node, plugin=None):
        super(ControlPanelStack, self).__init__()

        self.node = node

        self.stack = QStackedWidget(self)

        self.controlPanelWidget = ControlPanelWidget(node, plugin=self)
        # self.setupWidget = SetupWidget(node,plugin= self)

        self.stack.addWidget(self.controlPanelWidget)

        hbox = QGridLayout(self)
        hbox.setContentsMargins(0, 0, 0, 0)
        hbox.addWidget(self.stack)

        self.stack.setCurrentIndex(0)

        self.setLayout(hbox)

    def goToSettings(self, dataFilePath=None):
        if hasattr(self, 'setupWidget'):
            self.stack.removeWidget(self.setupWidget)
        self.setupWidget = SetupWidget(self.node,plugin= self, dataFilePath=dataFilePath)
        self.stack.addWidget(self.setupWidget)
        self.stack.setCurrentWidget(self.setupWidget)

    def goToDeletedRobotScreen(self):
        self.deletedRobotScreenWidget = DeletedRobotScreenWidget(stackWidget =self)
        self.stack.addWidget(self.deletedRobotScreenWidget)
        self.stack.setCurrentWidget( self.deletedRobotScreenWidget)

    def onDeletedRobotScreenReturn(self, data):
        # self.controlPanelWidget.initializeRobotsOptions()
        self.controlPanelWidget.setRobotOnScreen(data)
        self.stack.setCurrentWidget(self.controlPanelWidget)
        self.stack.removeWidget(self.deletedRobotScreenWidget)
        self.deletedRobotScreenWidget.deleteLater()
        self.deletedRobotScreenWidget = None