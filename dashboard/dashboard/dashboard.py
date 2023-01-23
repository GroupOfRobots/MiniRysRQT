from rqt_gui_py.plugin import Plugin
from .dashboard_stack import DashboardStack
from .dashboard_widget import DashboardWidget
from shared.utils.serial_number import setWidgetSerialNumber


class Dashboard(Plugin):
    def __init__(self, context):
        super(Dashboard, self).__init__(context)
        self.setObjectName('DashboardPanel')

        self._stack = DashboardStack(node=context.node)
        self._stack.setWindowTitle('Dashboard Panel')

        setWidgetSerialNumber(context, self._stack)

        context.add_widget(self._stack)

    def shutdown_plugin(self):
        print("shutdown_plugin")
        print(self.parent())
        print(self.children())
        # self.context.close_plugin()
        # self.context.remove_widget(self._stack)
        # print(self.parent().serialNumber())
        DashboardWidget.onDestroyed()
        print(self.parent().parent())
        print(self.parent().children())
        print(self.parent().parent().children())
        # a=self.parent().parent()
        # a.closePlugin()
