# This Python file uses the following encoding: utf-8

from setup_panel.setup_widget import SetupWidget
from .control_panel_widget import ControlPanelWidget
from shared.stack_widget.stack_widget import StackWidget

class ControlPanelStack(StackWidget):
    def __init__(self):
        super(ControlPanelStack, self).__init__()

        self.mainChildWidget = ControlPanelWidget( stack=self)
        self.stack.addWidget(self.mainChildWidget)

    def goToSettings(self, dataFilePath=None):
        if hasattr(self, 'setupWidget'):
            self.stack.removeWidget(self.setupWidget)
        self.setupWidget = SetupWidget(stack = self, dataFilePath=dataFilePath)
        self.stack.addWidget(self.setupWidget)
        self.stack.setCurrentWidget(self.setupWidget)
