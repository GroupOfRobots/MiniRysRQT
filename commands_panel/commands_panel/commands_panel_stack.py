# This Python file uses the following encoding: utf-8

from shared.stack_widget.stack_widget import StackWidget
from .commands_panel_widget import CommandsPanelWidget


class CommandsPanelStack(StackWidget):
    def __init__(self, node=None):
        super(CommandsPanelStack, self).__init__()

        self.mainChildWidget = CommandsPanelWidget(stack=self, node=node)
        self.stack.addWidget(self.mainChildWidget)
