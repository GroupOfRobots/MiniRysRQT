# This Python file uses the following encoding: utf-8

from shared.stack_widget.stack_widget import StackWidget
from .dashboard_distance_sensors_widget import DashboardDistanceSensorsWidget

class DashboardStack(StackWidget):
    def __init__(self, node):
        super(DashboardStack, self).__init__()

        self.mainChildWidget = DashboardDistanceSensorsWidget(stack=self, node=node)
        self.stack.addWidget(self.mainChildWidget)
