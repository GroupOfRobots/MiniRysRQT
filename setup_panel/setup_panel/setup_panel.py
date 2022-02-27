from rqt_gui_py.plugin import Plugin
from .setup_dashboard_stack import SetupDashboardStackWidget

class SetupPanel(Plugin):
    def __init__(self, context):
        super(SetupPanel, self).__init__(context)
        self._widget = SetupDashboardStackWidget()
        self._widget.setObjectName('SetupPanel')

        self._widget.setWindowTitle('Setup Panel')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)