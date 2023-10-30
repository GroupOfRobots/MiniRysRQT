# This Python file uses the following encoding: utf-8

import os

from ament_index_python import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget


class CommandElementWidget(QWidget):
    def __init__(self, widget=None, command=None):
        super(CommandElementWidget, self).__init__()

        self.widget = widget
        self.command = command

        self.loadUi()

        if self.command:
            self.setupCommand()

        self.commandDeleteButtonUI.clicked.connect(lambda: self.widget.deleteCommand(self))
        self.commandNameLineEditUI.textChanged.connect(self.setToolTip)

    def loadUi(self):
        _, packagePath = get_resource('packages', 'setup_panel')
        uiFile = os.path.join(packagePath, 'share', 'setup_panel', 'resource', 'command_element.ui')
        loadUi(uiFile, self)

    def setupCommand(self):
        commandName = self.command.get('commandName')
        self.commandNameLineEditUI.setText(commandName)
        self.setToolTip(commandName)

        command = self.command.get('command')
        self.commandTextEditUI.setPlainText(command)
        executeViaSsh = self.command.get('executeViaSsh')
        self.executeViaSshCheckBoxUI.setChecked(executeViaSsh)

    def setToolTip(self, toolTip):
        self.commandNameLineEditUI.setToolTip(toolTip)

    def returnCommand(self):
        return {'commandName': self.commandNameLineEditUI.text(),
                'command': self.commandTextEditUI.toPlainText(),
                'executeViaSsh': self.executeViaSshCheckBoxUI.isChecked()}
