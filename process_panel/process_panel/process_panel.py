from rqt_gui_py.plugin import Plugin
from .process_panel_stack import ProcessPanelStack
from python_qt_binding.QtCore import pyqtSignal

from shared.utils.serial_number import setWidgetSerialNumber

class ProcessPanel(Plugin):
    closePanelSignal = pyqtSignal(bool, name="closePanelSignal")

    def __init__(self, context):
        super(ProcessPanel, self).__init__(context)
        self.name = 'ProcessPanel' + str(context.serial_number())
        self.setObjectName(self.name)

        self._stack = ProcessPanelStack(node=context.node, processPanel=self)
        self._stack.setWindowTitle('Process Panel')

        setWidgetSerialNumber(context, self._stack)

        context.add_widget(self._stack)

    def shutdown_plugin(self):
        self.closePanelSignal.emit(True)
