from rqt_gui_py.plugin import Plugin

from shared.utils.serial_number import setWidgetSerialNumber
from .control_panel_stack import ControlPanelStack

class ControlPanel(Plugin):
    def __init__(self, context):
        super(ControlPanel, self).__init__(context)
        self.setObjectName('ControlPanel')

        self._stack = ControlPanelStack(node=context.node)
        self._stack.setWindowTitle('Control Panel')

        setWidgetSerialNumber(context, self._stack)

        context.add_widget(self._stack)
