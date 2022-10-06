from rqt_gui_py.plugin import Plugin
from .fan_panel_stack import FanPanelStack
from .fan_panel_widget import FanPanelWidget
from python_qt_binding.QtCore import pyqtSignal


class FanPanel(Plugin):
    fanPanelCounter = 0
    closePanelSignal = pyqtSignal(bool, name="closePanelSignal")

    def __init__(self, context):
        super(FanPanel, self).__init__(context)
        self.name = 'FanPanel' + str(context.serial_number())
        self.setObjectName(self.name)

        self._widget = FanPanelStack(node=context.node, fanPanel=self)
        self._widget.setWindowTitle('Fan Panel')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)
        FanPanel.fanPanelCounter += 1

    def shutdown_plugin(self):
        FanPanel.fanPanelCounter -= 1
        if FanPanel.fanPanelCounter == 0:
            self.closePanelSignal.emit(True)
