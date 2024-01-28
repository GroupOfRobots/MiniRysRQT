from python_qt_binding.QtCore import pyqtSignal
from shared.base_plugin.base_plugin import BasePlugin

from .fan_panel_stack import FanPanelStack


class FanPanel(BasePlugin):
    def __init__(self, context):
        super(FanPanel, self).__init__(context)
        self.name = 'FanPanel' + str(context.serial_number())
        self.setObjectName(self.name)

        stack = FanPanelStack(node=context.node, fanPanel=self)
        self.setStackWidget(stack, 'Fan Panel')
