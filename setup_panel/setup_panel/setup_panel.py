from rqt_gui_py.plugin import Plugin
from .setup import SetupWidget
from .setup_dashboard import SetupDashboardWidget


class SetupPanel(Plugin):

    def __init__(self, context):
        super(SetupPanel, self).__init__(context)
        self._node = context.node
        self.setObjectName('Test')

        # self._widget = ControlPanelWidget(context.node, self)
        # self._widget = SetupWidget(context.node, self)
        self._widget = SetupDashboardWidget(context.node, self)

        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)
