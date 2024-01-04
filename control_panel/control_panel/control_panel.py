from .control_panel_stack import ControlPanelStack

from shared.base_plugin.base_plugin import BasePlugin

class ControlPanel(BasePlugin):
    def __init__(self, context):
        super(ControlPanel, self).__init__(context)
        self.setObjectName('ControlPanel')

        stack = ControlPanelStack(node=context.node)
        self.setStackWidget(stack, 'Control Panel')
