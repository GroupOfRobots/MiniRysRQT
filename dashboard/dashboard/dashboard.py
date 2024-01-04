from rqt_gui_py.plugin import Plugin
from shared.base_plugin.base_plugin import BasePlugin

from .dashboard_stack import DashboardStack


class Dashboard(BasePlugin):
    def __init__(self, context):
        super(Dashboard, self).__init__(context)
        self.setObjectName('DashboardPanel')

        stack = DashboardStack(node=context.node)
        self.setStackWidget(stack, 'Dashboard Panel')
