from rqt_gui_py.plugin import Plugin
from .setup_dashboard_stack import SetupDashboardStackWidget
from shared.utils.serial_number import setWidgetSerialNumber

class SetupPanel(Plugin):
    def __init__(self, context):
        super(SetupPanel, self).__init__(context)
        self._stack = SetupDashboardStackWidget()
        self._stack.setObjectName('SetupPanel')
        self._stack.setWindowTitle('Setup Panel')

        setWidgetSerialNumber(context, self._stack)

        context.add_widget(self._stack)