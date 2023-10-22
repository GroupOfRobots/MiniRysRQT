from rqt_gui_py.plugin import Plugin

from .joystick_stack import JoystickStack
from shared.utils.serial_number import setWidgetSerialNumber


class Joystick(Plugin):
    def __init__(self, context):
        super(Joystick, self).__init__(context)
        self._node = context.node

        self.setObjectName('Joystick')

        self._stack = JoystickStack(self._node)
        self._stack.setWindowTitle('Joystick')

        setWidgetSerialNumber(context, self._stack)

        context.add_widget(self._stack)
