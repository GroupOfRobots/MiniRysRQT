from rqt_gui_py.plugin import Plugin
from .commands_panel_stack import CommandsPanelStack

class CommandsPanel(Plugin):
    def __init__(self, context):
        super(CommandsPanel, self).__init__(context)
        self.setObjectName('CommandsPanel')

        self._widget = CommandsPanelStack(context.node)
        self._widget.setWindowTitle('Commands Panel')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)
