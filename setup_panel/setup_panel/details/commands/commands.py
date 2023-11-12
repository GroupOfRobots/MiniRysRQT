# This Python file uses the following encoding: utf-8

from python_qt_binding.QtWidgets import QWidget

from .command_element import CommandElementWidget


class Commands(QWidget):
    def __init__(self, widget=None, command=None):
        super(Commands, self).__init__()

        self.widget = widget
        self.command = command
        self.setData(self.widget.data)

    def setData(self, data):
        self.data = data

        self.widget.addNewCommandButtonUI.clicked.connect(self.addNewCommand)
        commands = self.data.get('commands', [])
        for command in commands:
            commandElement = CommandElementWidget(self, command)
            self.widget.commandElementsBoxUI.addWidget(commandElement)

    def addNewCommand(self):
        commandElement = CommandElementWidget(self, None)
        self.widget.commandElementsBoxUI.addWidget(commandElement)

    def deleteCommand(self, commandElement):
        self.widget.commandElementsBoxUI.removeWidget(commandElement)
        commandElement.deleteLater()
        commandElement = None

    def saveCommands(self):
        print(self.widget)
        self.widget.data['commands'] = []
        for index in range(0, self.widget.commandElementsBoxUI.count()):
            commandElement = self.widget.commandElementsBoxUI.itemAt(index).widget()
            self.widget.data['commands'].append(commandElement.returnCommand())
