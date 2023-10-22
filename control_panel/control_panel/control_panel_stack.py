# This Python file uses the following encoding: utf-8

from shared.stack_widget.stack_widget import StackWidget

from .control_panel_widget import ControlPanelWidget


class ControlPanelStack(StackWidget):
    def __init__(self, node=None):
        super(ControlPanelStack, self).__init__(node=node, constructor=ControlPanelWidget)
