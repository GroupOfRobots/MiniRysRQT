# This Python file uses the following encoding: utf-8
import os, os.path

from python_qt_binding.QtWidgets import QWidget, QStackedWidget, QHBoxLayout, QListWidget, QGridLayout, QLayout



# import QObjectClass
# import PyQt5.QtWidgets


from .dashboard_element import DashboardElementWidget
from .setup import SetupWidget
from .setup_dashboard import SetupDashboardWidget


class SetupDashboardStackWidget(QWidget):
    def __init__(self, node, plugin=None):
        super(SetupDashboardStackWidget, self).__init__()

        # self.setObjectName("aaa")

        self.stack = QStackedWidget(self)

        self.node =node

        self.controlPanelWidget = SetupDashboardWidget(node, plugin = self)

        self.stack.addWidget(self.controlPanelWidget)

        hbox = QGridLayout(self)
        hbox.addWidget(self.stack)

        self.stack.setCurrentIndex(0)

        self.setLayout(hbox)

    def goToSettings(self, fileName=None):
        if hasattr(self, 'setupWidget'):
            self.stack.removeWidget(self.setupWidget)
        self.setupWidget = SetupWidget(self.node, fileName=fileName,plugin=self)
        self.stack.addWidget(self.setupWidget)
        self.stack.setCurrentIndex(1)
