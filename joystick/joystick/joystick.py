from shared.base_plugin.base_plugin import BasePlugin

from .joystick_stack import JoystickStack


class Joystick(BasePlugin):
    def __init__(self, context):
        super(Joystick, self).__init__(context)
        self.setObjectName('Joystick')

        stack = JoystickStack(context.node)
        self.setStackWidget(stack, 'Joystick')
