# This Python file uses the following encoding: utf-8

from python_qt_binding.QtCore import Qt,pyqtSignal
from python_qt_binding.QtGui import QColor, QFont,QCursor,QCursor,QTextCharFormat

from python_qt_binding.QtWidgets import QPlainTextEdit, QVBoxLayout, QWidget,QPushButton

from shared.base_widget.base_widget import BaseWidget
from shared.enums import PackageNameEnum

from .command_execute_element import CommandExecuteElementWidget


class CommandsPanelWidget(BaseWidget):
    commandOutputSignal = pyqtSignal(object, name="commandExecutionOutput")

    def __init__(self, stack=None, node=None):
        super(CommandsPanelWidget, self).__init__(stack, PackageNameEnum.CommandsPanel)

        self.commandOutputSignal.connect(self.onCommandOutputSignal)

        self.setRobotOnScreen()

        self.logFormat = QTextCharFormat()
        self.clearAllConsoleButtonUI.clicked.connect(self.clearAllTabs)
        self.closeAllConsoleButtonUI.clicked.connect(self.closeAllTabs)
        self.settingsButtonUI.clicked.connect(self.settingsClicked)

        self.logsTabWidgetUI.setTabsClosable(True)
        self.logsTabWidgetUI.tabCloseRequested.connect(self.tabCloseRequested)

        self.tabDictionary = {}

    def clearAllTabs(self):
        for plainTextEdit in self.tabDictionary.values():
            plainTextEdit.clear()

    def closeAllTabs(self):
        self.tabDictionary.clear()
        self.logsTabWidgetUI.clear()

    def addTab(self, tabName):
        plainTextEdit = QPlainTextEdit()
        plainTextEdit.setReadOnly(True)
        button = QPushButton("CLEAR")
        button.setCursor(QCursor(Qt.PointingHandCursor))
        button.clicked.connect(lambda: plainTextEdit.clear())
        button.setStyleSheet("border: 1px solid black;\nborder-radius: 5px;\npadding: 5px;\nbackground: gray;")
        button.setMinimumWidth(150)
        button.setMaximumWidth(150)

        # Create a widget to hold the plain text edit (optional)
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.addWidget(button)
        layout.addWidget(plainTextEdit)
        self.tabDictionary[tabName] = plainTextEdit

        self.logsTabWidgetUI.insertTab(0, widget, tabName)
        self.logsTabWidgetUI.setCurrentIndex(0)

    def tabCloseRequested(self, index):
        self.tabDictionary.pop(self.logsTabWidgetUI.tabText(index))

        self.logsTabWidgetUI.removeTab(index)

    def initializeRobotSettings(self):
        for index in range(self.commandsBoxLayoutUI.count()):
            self.commandsBoxLayoutUI.itemAt(index).widget().deleteLater()

        commands = self.data.get('commands', [])

        for command in commands:
            element = CommandExecuteElementWidget(command, self.data, self.commandOutputSignal)
            self.commandsBoxLayoutUI.addWidget(element)

    def onCommandOutputSignal(self, commandOutput):
        [command, output, errors] = commandOutput

        if self.tabDictionary.get(command) is None:
            self.addTab(command)

        if (len(command)):
            self.logFormat.setFontWeight(QFont.Bold)
            self.insertLog("black", "COMMAND:", command)
            self.logFormat.setFontWeight(QFont.Normal)

            self.insertLog("blue", command + "\n", command)

        if len(output):
            self.insertLog("black", output, command)

        if len(errors):
            self.insertLog("red", errors, command)

    def insertLog(self, colorName, log, command):

        color = QColor(colorName)
        self.logFormat.setForeground(color)
        plainTextEdit = self.tabDictionary.get(command)
        plainTextEdit.setCurrentCharFormat(self.logFormat)
        plainTextEdit.insertPlainText(log + "\n")

    def settingsClicked(self):
        self.stack.goToSettings(self.dataFilePath)
