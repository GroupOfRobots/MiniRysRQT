# This Python file uses the following encoding: utf-8

from shared.utils.load_ui_file import loadUiFile

from python_qt_binding.QtWidgets import QWidget

from shared.enums import PackageNameEnum, packageNameToUIFileMap


class CommandElementWidget(QWidget):
    def __init__(self, widget=None, command=None):
        super(CommandElementWidget, self).__init__()

        self.widget = widget
        self.command = command

        loadUiFile(self, PackageNameEnum.SetupPanel, 'command_element.ui')

        if self.command:
            self.setupCommand()

        self.commandDeleteButtonUI.clicked.connect(lambda: self.widget.deleteCommand(self))
        self.commandNameLineEditUI.textChanged.connect(self.setToolTip)

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
