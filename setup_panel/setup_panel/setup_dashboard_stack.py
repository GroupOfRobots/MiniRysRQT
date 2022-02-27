# This Python file uses the following encoding: utf-8


from python_qt_binding.QtWidgets import QStackedWidget

from .setup_dashboard_widget import SetupDashboardWidget
from .setup_widget import SetupWidget

from shared.stack_widget.stack_widget import StackWidget


# import QObjectClass
# import PyQt5.QtWidgets

class SetupDashboardStackWidget(StackWidget):
    def __init__(self, node, plugin=None, context=None):
        super(SetupDashboardStackWidget, self).__init__()

        self.context = context

        self.node = node

        self.mainChildWidget = SetupDashboardWidget(stack=self)
        self.stack.addWidget(self.mainChildWidget)

    def goToSettings(self, dataFilePath=None):
        if hasattr(self, 'setupWidget'):
            self.stack.removeWidget(self.setupWidget)
        self.setupWidget = SetupWidget(self.node, dataFilePath=dataFilePath, plugin=self, context=self.context)
        self.stack.addWidget(self.setupWidget)
        self.stack.setCurrentIndex(1)
