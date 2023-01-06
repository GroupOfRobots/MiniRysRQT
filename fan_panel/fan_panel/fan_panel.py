from rqt_gui_py.plugin import Plugin
from .fan_panel_stack import FanPanelStack
from .fan_panel_widget import FanPanelWidget
from python_qt_binding.QtCore import pyqtSignal

from shared.utils.serial_number import setWidgetSerialNumber

class FanPanel(Plugin):
    fanPanelCounter = 0
    closePanelSignal = pyqtSignal(bool, name="closePanelSignal")

    def __init__(self, context):
        super(FanPanel, self).__init__(context)
        self.name = 'FanPanel' + str(context.serial_number())
        self.setObjectName(self.name)

        self._stack = FanPanelStack(node=context.node, fanPanel=self)
        self._stack.setWindowTitle('Fan Panel')

        setWidgetSerialNumber(context, self._stack)

        context.add_widget(self._stack)
        FanPanel.fanPanelCounter += 1

    def shutdown_plugin(self):
        FanPanel.fanPanelCounter -= 1
        if FanPanel.fanPanelCounter == 0:
            self.closePanelSignal.emit(True)
