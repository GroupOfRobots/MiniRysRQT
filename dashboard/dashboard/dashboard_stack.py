# This Python file uses the following encoding: utf-8
import os

from python_qt_binding.QtWidgets import QWidget, QStackedWidget,QHBoxLayout,QGridLayout,QStackedLayout
from ament_index_python import get_resource
from python_qt_binding import loadUi

import json

from shared.utils.utils import initializeRobotsOptions
from shared.inner_communication import innerCommunication

from .dashboard_widget import DashboardWidget


class DashboardStack(QWidget):
    def __init__(self, node, plugin=None, fileName=None):
        super(DashboardStack, self).__init__()

        self.stack = QStackedWidget(self)
        self.dashboardWidget = DashboardWidget(node, plugin=self)

        self.stack.addWidget(self.dashboardWidget)

        # layout = QGridLayout(self)
        # layout.setContentsMargins(0, 0, 0, 0)
        # layout.addWidget(self.stack)
        stackedLayout = QStackedLayout()
        stackedLayout.addWidget(self.stack)

        self.stack.setCurrentIndex(0)

        self.setLayout(stackedLayout)
        # self.setLayout(layout)
        # self.stack.show()
        # print('aaaaaaaaaaaaaaaaaaaaaaaaaaaa')


