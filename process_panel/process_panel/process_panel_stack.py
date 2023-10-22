# This Python file uses the following encoding: utf-8

from shared.stack_widget.stack_widget import StackWidget

from .process_panel_widget import ProcessPanelWidget


class ProcessPanelStack(StackWidget):
    def __init__(self, node=None, processPanel=None):
        super(ProcessPanelStack, self).__init__(node=node, constructor=ProcessPanelWidget, mainPanel=processPanel)
