from rqt_gui_py.plugin import Plugin
from shared.base_plugin.base_plugin import BasePlugin

from .commands_panel_stack import CommandsPanelStack

class CommandsPanel(Plugin):

class CommandsPanel(BasePlugin):
    def __init__(self, context):
        super(CommandsPanel, self).__init__(context)
        self.setObjectName('CommandsPanel')

        stack = CommandsPanelStack(node=context.node)
        self.setStackWidget(stack, 'Commands Panel')
