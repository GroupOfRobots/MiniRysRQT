from rqt_gui_py.plugin import Plugin
from .dashboard_widget import DashboardWidget


class Dashboard(Plugin):
    def __init__(self, context):
        super(Dashboard, self).__init__(context)
        self._node = context.node
        self.setObjectName('Test')

        self._widget = DashboardWidget(context.node, self)

        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)
