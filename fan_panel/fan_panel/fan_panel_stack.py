# This Python file uses the following encoding: utf-8

from .fan_panel_widget import FanPanelWidget
from shared.stack_widget.stack_widget import StackWidget

class FanPanelStack(StackWidget):
    def __init__(self, node=None, fanPanel=None):
        super(FanPanelStack, self).__init__()

        self.mainChildWidget = FanPanelWidget( stack=self, node=node, fanPanel=fanPanel)
        self.stack.addWidget(self.mainChildWidget)

