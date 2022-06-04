# This Python file uses the following encoding: utf-8

from shared.inner_communication import innerCommunication
from python_qt_binding.QtWidgets import QPushButton, QWidget,QLineEdit,QTableWidget,QTableWidgetItem,QMessageBox
from ament_index_python import get_resource
from python_qt_binding import loadUi

import json
import os

class CommandElementWidget(QWidget):
    def __init__(self, fileName=None, stack=None):
        super(CommandElementWidget, self).__init__()

        self.loadUi()



    def loadUi(self):
        _, packagePath = get_resource('packages', 'setup_panel')
        uiFile = os.path.join(packagePath, 'share', 'setup_panel', 'resource', 'command_element.ui')
        loadUi(uiFile, self)


