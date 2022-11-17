from rqt_gui_py.plugin import Plugin
from .dashboard_distance_sensors_stack import DashboardStack
from .dashboard_distance_sensors_widget import DashboardDistanceSensorsWidget

class DashboardDistanceSensors(Plugin):
    def __init__(self, context):
        super(DashboardDistanceSensors, self).__init__(context)
        self._node = context.node
        self.context= context
        self.setObjectName('Test')

        self._stack = DashboardStack(context.node)

        if context.serial_number() > 1:
            self._stack.setWindowTitle(
                self._stack.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._stack)

    def shutdown_plugin(self):
        print("aaaaaaaaaaaa")
        # DashboardDistanceSensorsWidget.onDestroyed()