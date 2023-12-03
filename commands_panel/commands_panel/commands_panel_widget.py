# This Python file uses the following encoding: utf-8

from python_qt_binding.QtCore import Qt, pyqtSignal
from python_qt_binding.QtGui import QColor, QFont, QCursor, QTextCharFormat
from python_qt_binding.QtWidgets import QPlainTextEdit, QVBoxLayout, QWidget, QPushButton
from shared.base_widget.base_widget import BaseWidget
from shared.enums import PackageNameEnum

from .command_execute_element import CommandExecuteElementWidget


class CommandsPanelWidget(BaseWidget):
    # Extracted outside constructor due the following error
    # https://stackoverflow.com/questions/2970312/pyqt4-qtcore-pyqtsignal-object-has-no-attribute-connect
    commandOutputSignal = pyqtSignal(object, name="commandExecutionOutput")

    def __init__(self, stack=None, node=None):
        super(CommandsPanelWidget, self).__init__(stack=stack, packageName=PackageNameEnum.CommandsPanel, node=node)

        self.commandOutputSignal.connect(self.onCommandOutputSignal)

        self.setRobotOnScreen()

        self.logFormat = QTextCharFormat()
        self.clearAllConsoleButtonUI.clicked.connect(self.clearAllTabs)
        self.closeAllConsoleButtonUI.clicked.connect(self.closeAllTabs)

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
        [commandName, command, output, errors, firstLog] = commandOutput

        if firstLog and self.tabDictionary.get(commandName) is None:
            self.addTab(commandName)

        if firstLog and len(command):
            self.logFormat.setFontWeight(QFont.Bold)
            self.insertLog("black", "COMMAND:", commandName)
            self.logFormat.setFontWeight(QFont.Normal)

            self.insertLog("blue", command + "\n", commandName)

        if len(output):
            self.insertLog("black", output, commandName)

        if len(errors):
            self.insertLog("red", errors, commandName)

    def insertLog(self, colorName, log, commandName):
        color = QColor(colorName)
        self.logFormat.setForeground(color)
        plainTextEdit = self.tabDictionary.get(commandName)
        plainTextEdit.setCurrentCharFormat(self.logFormat)
        plainTextEdit.insertPlainText(log + "\n")

