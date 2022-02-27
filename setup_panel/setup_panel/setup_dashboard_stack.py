# This Python file uses the following encoding: utf-8

from .setup_dashboard_widget import SetupDashboardWidget
from .setup_widget import SetupWidget

from shared.stack_widget.stack_widget import StackWidget

class SetupDashboardStackWidget(StackWidget):
    def __init__(self):
        super(SetupDashboardStackWidget, self).__init__()

        self.mainChildWidget = SetupDashboardWidget(stack=self)
        self.stack.addWidget(self.mainChildWidget)

    def goToSettings(self, dataFilePath=None):
        if hasattr(self, 'setupWidget'):
            self.stack.removeWidget(self.setupWidget)
        self.setupWidget = SetupWidget(dataFilePath=dataFilePath, stack=self)
        self.stack.addWidget(self.setupWidget)
        self.stack.setCurrentIndex(1)
