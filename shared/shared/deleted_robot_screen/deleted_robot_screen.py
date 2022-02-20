from rqt_gui_py.plugin import Plugin

from .deleted_robot_screen_widget import DeletedRobotScreenWidget

class DeletedRobotScreen(Plugin):
    def __init__(self, context):
        super(DeletedRobotScreen, self).__init__(context)
        self._node = context.node

        self._widget = DeletedRobotScreenWidget(context.node, self)

        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)
