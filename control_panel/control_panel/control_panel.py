from rqt_gui_py.plugin import Plugin
from .control_panel_stack import ControlPanelStack

class ControlPanel(Plugin):
    def __init__(self, context):
        super(ControlPanel, self).__init__(context)
        self.setObjectName('ControlPanel')

        self._widget = ControlPanelStack(node=context.node)
        self._widget.setWindowTitle('Control Panel')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)
