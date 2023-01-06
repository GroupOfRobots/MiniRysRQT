from rqt_gui_py.plugin import Plugin
from .dashboard_distance_sensors_stack import DashboardStack
from shared.utils.serial_number import setWidgetSerialNumber

class DashboardDistanceSensors(Plugin):
    def __init__(self, context):
        super(DashboardDistanceSensors, self).__init__(context)
        self._node = context.node
        self.context= context
        self.setObjectName('Test')

        self._stack = DashboardStack(context.node)

        setWidgetSerialNumber(context, self._stack)

        context.add_widget(self._stack)

    def shutdown_plugin(self):
        print("aaaaaaaaaaaa")
        # DashboardDistanceSensorsWidget.onDestroyed()