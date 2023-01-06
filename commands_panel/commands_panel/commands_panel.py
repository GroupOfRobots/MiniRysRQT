from rqt_gui_py.plugin import Plugin
from .commands_panel_stack import CommandsPanelStack
from shared.utils.serial_number import setWidgetSerialNumber

class CommandsPanel(Plugin):
    def __init__(self, context):
        super(CommandsPanel, self).__init__(context)
        self.setObjectName('CommandsPanel')

        self._stack = CommandsPanelStack(context.node)
        self._stack.setWindowTitle('Commands Panel')

        setWidgetSerialNumber(context, self._stack)

        context.add_widget(self._stack)
