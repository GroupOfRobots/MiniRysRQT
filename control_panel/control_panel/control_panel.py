from rqt_gui_py.plugin import Plugin
from .control_panel_stack import ControlPanelStack

class ControlPanel(Plugin):
    def __init__(self, context):
        super(ControlPanel, self).__init__(context)
        self._node = context.node
        self.setObjectName('Test')

        self._widget = ControlPanelStack(context.node, self)

        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)
