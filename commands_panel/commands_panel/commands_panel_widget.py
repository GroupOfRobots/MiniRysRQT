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
from python_qt_binding.QtGui import QColor, QFont


class CommandsPanelWidget(BaseWidget):
    commandOutputSignal = pyqtSignal(object, name="commandExecutionOutput")

    def __init__(self, stack=None, node=None):
        super(CommandsPanelWidget, self).__init__(stack)

        # self.loadUI()
        # self.initializeRobotsOptions()
        self.commandOutputSignal.connect(self.onCommandOutputSignal)

        self.comboBox.currentIndexChanged.connect(self.setRobotOnScreen)
        self.setRobotOnScreen()

        self.logFormat = self.logsPlainTextEditUI.currentCharFormat()
        self.clearConsoleButtonUI.clicked.connect(self.clearConsole)
        self.settingsButtonUI.clicked.connect(self.settingsClicked)

    def initializeRobotSettings(self):
        for index in range(self.commandsBoxLayoutUI.count()):
            self.commandsBoxLayoutUI.itemAt(index).widget().deleteLater()

        commands = self.data.get('commands', [])

        for command in commands:
            element = CommandExecuteElementWidget(command, self.data, self.commandOutputSignal)
            self.commandsBoxLayoutUI.addWidget(element)

    def loadUI(self):
        _, packagePath = get_resource('packages', 'commands_panel')
        uiFile = os.path.join(packagePath, 'share', 'commands_panel', 'resource', 'commands_panel.ui')
        loadUi(uiFile, self)

    def clearConsole(self):
        self.logsPlainTextEditUI.clear()

    def onCommandOutputSignal(self, commandOutput):
        [command, output, errors] = commandOutput

        self.logFormat.setFontWeight(QFont.Bold)
        self.insertLog("black", "COMMAND:")
        self.logFormat.setFontWeight(QFont.Normal)

        if (len(command)):
            self.insertLog("blue", command + "\n")

        if len(output):
            self.insertLog("black", output)

        if len(errors):
            self.insertLog("red", errors)

        self.logsPlainTextEditUI.insertPlainText("\n")

    def insertLog(self, colorName, log):
        color = QColor(colorName)
        self.logFormat.setForeground(color)
        self.logsPlainTextEditUI.setCurrentCharFormat(self.logFormat)
        self.logsPlainTextEditUI.insertPlainText(log + "\n")

    def settingsClicked(self):
        self.stack.goToSettings(self.dataFilePath)
