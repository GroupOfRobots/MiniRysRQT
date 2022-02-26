# This Python file uses the following encoding: utf-8
import os, os.path



import asyncio
import can



from python_qt_binding.QtWidgets import QWidget, QStackedWidget, QStackedLayout, QListWidget, QGridLayout, QLayout



# import QObjectClass
# import PyQt5.QtWidgets


from .dashboard_element import DashboardElementWidget
from .setup_widget import SetupWidget
from .setup_dashboard import SetupDashboardWidget


class SetupDashboardStackWidget(QWidget):
    def __init__(self, node, plugin=None, context=None):
        super(SetupDashboardStackWidget, self).__init__()

        self.context= context

        # self.setObjectName("aaa")

        self.stack = QStackedWidget(self)

        self.node =node

        self.controlPanelWidget = SetupDashboardWidget(node, plugin = self,context=context)

        self.stack.addWidget(self.controlPanelWidget)

        hbox = QStackedLayout(self)
        hbox.addWidget(self.stack)

        self.stack.setCurrentIndex(0)

        self.setLayout(hbox)

    def goToSettings(self, dataFilePath=None):
        if hasattr(self, 'setupWidget'):
            self.stack.removeWidget(self.setupWidget)
        self.setupWidget = SetupWidget(self.node, dataFilePath=dataFilePath,plugin=self,context=self.context)
        self.stack.addWidget(self.setupWidget)
        self.stack.setCurrentIndex(1)
