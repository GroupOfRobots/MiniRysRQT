from rqt_gui_py.plugin import Plugin
from .fan_panel_stack import FanPanelStack

class FanPanel(Plugin):
    def __init__(self, context):
        super(FanPanel, self).__init__(context)
        self.setObjectName('FanPanel')

        self._widget = FanPanelStack(context.node)
        self._widget.setWindowTitle('Fan Panel')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)
