from rqt_gui_py.plugin import Plugin
from .camera_panel_stack import CameraPanelStack
from shared.utils.serial_number import setWidgetSerialNumber

class CameraPanel(Plugin):
    def __init__(self, context):
        super(CameraPanel, self).__init__(context)
        self.setObjectName('CameraPanel')

        self._stack = CameraPanelStack(context.node)
        self._stack.setWindowTitle('Camera Panel')

        setWidgetSerialNumber(context, self._stack)

        context.add_widget(self._stack)
