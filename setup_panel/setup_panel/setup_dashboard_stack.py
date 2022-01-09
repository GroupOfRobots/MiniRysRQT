# This Python file uses the following encoding: utf-8
import os, os.path

from python_qt_binding.QtWidgets import QWidget, QStackedWidget,QHBoxLayout,QListWidget,QGridLayout,QLayout


# from PyQt5.QtWidgets import QVBoxLayout

from .dashboard_element import DashboardElementWidget
from .setup import SetupWidget
from .setup_dashboard import SetupDashboardWidget


class SetupDashboardStackWidget(QWidget):
    def __init__(self, node, plugin=None):
        super(SetupDashboardStackWidget, self).__init__()

        self.stack = QStackedWidget(self)

        self.controlPanelWidget = SetupDashboardWidget(node, self)
        # self.controlPanelWidget3 = SetupWidget(node, self)

        self.stack.addWidget(self.controlPanelWidget)
        # self.stack.addWidget(self.controlPanelWidget3)

        hbox = QGridLayout(self)
        hbox.addWidget(self.stack)

        self.stack.setCurrentIndex(0)

        self.setLayout(hbox)

    def goToSettings(self, fileName =  None):
        print('goToSettings')
        print(fileName)
        if hasattr(self,'setupWidget'):
            self.stack.removeWidget(self.setupWidget)
        self.setupWidget = SetupWidget(self,fileName=fileName)
        self.stack.addWidget(self.setupWidget)
        self.stack.setCurrentIndex(1)
