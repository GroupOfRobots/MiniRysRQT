# This Python file uses the following encoding: utf-8

from shared.stack_widget.stack_widget import StackWidget
from .camera_panel_widget import CameraPanelWidget


class CameraPanelStack(StackWidget):
    def __init__(self, node=None):
        super(CameraPanelStack, self).__init__()

        self.mainChildWidget = CameraPanelWidget(stack=self, node=node)
        self.stack.addWidget(self.mainChildWidget)
