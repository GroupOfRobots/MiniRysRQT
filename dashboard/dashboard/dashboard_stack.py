# This Python file uses the following encoding: utf-8
import os

from python_qt_binding.QtWidgets import QWidget, QStackedWidget, QHBoxLayout, QGridLayout, QStackedLayout
from ament_index_python import get_resource
from python_qt_binding import loadUi

import json

from shared.utils.utils import initializeRobotsOptions
from shared.inner_communication import innerCommunication
from shared.deleted_robot_screen.deleted_robot_screen import DeletedRobotScreenWidget

from .dashboard_widget import DashboardWidget


class DashboardStack(QWidget):
    def __init__(self, node, plugin=None, fileName=None):
        super(DashboardStack, self).__init__()

        self.stack = QStackedWidget(self)
        self.dashboardWidget = DashboardWidget(node, plugin=self, stack=self)

        self.stack.addWidget(self.dashboardWidget)

        stackedLayout = QStackedLayout()
        stackedLayout.addWidget(self.stack)

        # self.stack.setCurrentIndex(0)

        self.setLayout(stackedLayout)

    def goToDeletedRobotScreen(self):
        self.deletedRobotScreenWidget = DeletedRobotScreenWidget(stackWidget =self)
        self.stack.addWidget(self.deletedRobotScreenWidget)
        self.stack.setCurrentWidget( self.deletedRobotScreenWidget)

    def onDeletedRobotScreenReturn(self, data):
        # self.controlPanelWidget.initializeRobotsOptions()
        # self.controlPanelWidget.setRobotOnScreen(data)
        self.stack.setCurrentWidget(self.dashboardWidget)
        self.stack.removeWidget(self.deletedRobotScreenWidget)
        self.deletedRobotScreenWidget.deleteLater()
        self.deletedRobotScreenWidget = None
