# This Python file uses the following encoding: utf-8

from shared.stack_widget.stack_widget import StackWidget

from .details.setup_widget import SetupWidget
from .setup_dashboard_widget import SetupDashboardWidget


class SetupDashboardStackWidget(StackWidget):
    def __init__(self, node, panel):
        super(SetupDashboardStackWidget, self).__init__(node=node, constructor=SetupDashboardWidget, mainPanel=panel)
        self.node = node

    def goToSettings(self, dataFilePath=None, addMode=False):
        if hasattr(self, 'setupWidget'):
            self.stack.removeWidget(self.setupWidget)
        self.setupWidget = SetupWidget(dataFilePath=dataFilePath, stack=self, addMode=addMode, node=self.node)
        self.stack.addWidget(self.setupWidget)
        self.stack.setCurrentIndex(1)


    def initWidget(self):
        self.createWidget()
