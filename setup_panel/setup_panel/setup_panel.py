from shared.base_plugin.base_plugin import BasePlugin

from .setup_dashboard_stack import SetupDashboardStackWidget


class SetupPanel(BasePlugin):
    def __init__(self, context):
        super(SetupPanel, self).__init__(context)
        self.setObjectName('SetupPanel')

        stack = SetupDashboardStackWidget(node=context.node, panel=self)
        self.setStackWidget(stack, 'Setup Panel')
