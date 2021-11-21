from rqt_gui_py.plugin import Plugin

from .joystick1 import Joystick1


class Joystick(Plugin):

    def __init__(self, context):
        super(Joystick, self).__init__(context)
        self._node = context.node
        # self._logger = self._node.get_logger().get_child('rqt_rt_preempt.ros_rt_preempt.RosRtpreempt')

        super(Joystick, self).__init__(context)
        self.setObjectName('RosRtpreempt')

        self._widget = Joystick1(context.node, self)

        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)
