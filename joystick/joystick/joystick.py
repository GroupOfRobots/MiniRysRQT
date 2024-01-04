import sip
from rqt_gui_py.plugin import Plugin
from shared.utils.serial_number import setWidgetSerialNumber

from .joystick_stack import JoystickStack


class Joystick(Plugin):
    def __init__(self, context):
        super(Joystick, self).__init__(context)
        self._node = context.node

        self.setObjectName('Joystick')

        self._stack = JoystickStack(self._node)
        self._stack.setWindowTitle('Joystick')

        setWidgetSerialNumber(context, self._stack)

        context.add_widget(self._stack)

    def shutdown_plugin(self):
        self._stack.mainChildWidget.cleanup()
        sip.delete(self._stack.stack)
