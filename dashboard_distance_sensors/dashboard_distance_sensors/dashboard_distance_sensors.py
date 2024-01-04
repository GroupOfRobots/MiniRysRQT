from shared.base_plugin.base_plugin import BasePlugin

from .dashboard_distance_sensors_stack import DashboardStack


class DashboardDistanceSensors(BasePlugin):
    def __init__(self, context):
        super(DashboardDistanceSensors, self).__init__(context)
        self.setObjectName('DashboardDistanceSensors')

        stack = DashboardStack(node=context.node)
        self.setStackWidget(stack, 'Dashboard Distance Sensors')
