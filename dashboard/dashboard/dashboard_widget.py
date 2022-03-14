# This Python file uses the following encoding: utf-8
import os

from ament_index_python import get_resource
from python_qt_binding import loadUi
from shared.base_widget.base_widget import BaseWidget

class DashboardWidget(BaseWidget):
    def __init__(self, stack = None):
        super(DashboardWidget, self).__init__()
        BaseWidget.__init__(self, stack)

        self.loadUi()

        self.initializeRobotsOptions()

    def loadUi(self):
        _, packagePath = get_resource('packages', 'dashboard')
        uiFile = os.path.join(packagePath, 'share', 'dashboard', 'resource', 'dashboard.ui')
        loadUi(uiFile, self)

    def initializeSettings(self):
        print('initializeSettings')