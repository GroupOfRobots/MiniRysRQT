# This Python file uses the following encoding: utf-8

from shared.stack_widget.stack_widget import StackWidget

from .fan_panel_widget import FanPanelWidget


class FanPanelStack(StackWidget):
    def __init__(self, node=None, fanPanel=None):
        super(FanPanelStack, self).__init__(node=node, constructor=FanPanelWidget, mainPanel=fanPanel)
