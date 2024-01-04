from python_qt_binding.QtCore import pyqtSignal
from shared.base_plugin.base_plugin import BasePlugin

from .process_panel_stack import ProcessPanelStack


class ProcessPanel(BasePlugin):
    closePanelSignal = pyqtSignal(bool, name="closePanelSignal")

    def __init__(self, context):
        super(ProcessPanel, self).__init__(context)
        self.name = 'ProcessPanel' + str(context.serial_number())
        self.setObjectName(self.name)

        stack = ProcessPanelStack(node=context.node, processPanel=self)
        self.setStackWidget(stack, 'Process Panel')

    def shutdown_plugin(self):
        self.closePanelSignal.emit(True)
        super().shutdown_plugin()
