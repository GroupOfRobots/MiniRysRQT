# This Python file uses the following encoding: utf-8

from .control_panel_widget import ControlPanelWidget
from shared.stack_widget.stack_widget import StackWidget

class ControlPanelStack(StackWidget):
    def __init__(self):
        super(ControlPanelStack, self).__init__()

        self.mainChildWidget = ControlPanelWidget( stack=self)
        self.stack.addWidget(self.mainChildWidget)

