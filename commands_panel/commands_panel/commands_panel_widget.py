# This Python file uses the following encoding: utf-8
import json
import os

# from PyQt5.QtWidgets import QAbstractSpinBox
# from python_qt_binding.QtWidgets 
from python_qt_binding.QtWidgets import QAbstractSpinBox
from shared.base_widget.base_widget import BaseWidget

from std_msgs.msg import Float32
from ament_index_python import get_resource
from python_qt_binding import loadUi

from .command_execute_element import CommandExecuteElementWidget

from python_qt_binding.QtCore import pyqtSignal, QObject

class CommandsPanelWidget(BaseWidget):
    commandOutputSignal = pyqtSignal(object, name="commandExecutionOutput")

    def __init__(self, stack=None, node=None):
        super(CommandsPanelWidget, self).__init__()
        BaseWidget.__init__(self, stack)

        self.loadUI()
        self.initializeRobotsOptions()

        self.comboBox.currentIndexChanged.connect(self.setRobotOnScreen)
        self.setRobotOnScreen()

    def initializeRobotSettings(self):
        for index in range(self.commandsBoxLayoutUI.count()):
            self.commandsBoxLayoutUI.itemAt(index).widget().deleteLater()

        commands = self.data.get('commands', [])
        self.commandOutputSignal.connect(self.onCommandOutputSignal)


        for command in commands:
            element = CommandExecuteElementWidget(command, self.data, self.commandOutputSignal)
            self.commandsBoxLayoutUI.addWidget(element)

    def loadUI(self):
        _, packagePath = get_resource('packages', 'commands_panel')
        uiFile = os.path.join(packagePath, 'share', 'commands_panel', 'resource', 'commands_panel.ui')
        loadUi(uiFile, self)

    def onCommandOutputSignal(self, commandOutput):
        self.logsPlainTextEditUI.setPlainText(commandOutput)
