# This Python file uses the following encoding: utf-8

from .process_panel_widget import ProcessPanelWidget
from shared.stack_widget.stack_widget import StackWidget

class ProcessPanelStack(StackWidget):
    def __init__(self, node=None, processPanel=None):
        super(ProcessPanelStack, self).__init__()

        self.mainChildWidget = ProcessPanelWidget( stack=self, node=node, processPanel=processPanel)
        self.stack.addWidget(self.mainChildWidget)

