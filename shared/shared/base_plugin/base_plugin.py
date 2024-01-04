import gc

from rqt_gui_py.plugin import Plugin
from shared.utils.serial_number import setWidgetSerialNumber


class BasePlugin(Plugin):
    def __init__(self, context):
        super(BasePlugin, self).__init__(context)
        self.context = context

    def setStackWidget(self, stackWidget, title):
        self._stack = stackWidget
        self._stack.setWindowTitle(title)

        setWidgetSerialNumber(self.context, self._stack)

        self.context.add_widget(self._stack)

    def shutdown_plugin(self):
        self._stack.stack.removeWidget(self._stack.mainChildWidget)
        self._stack.mainChildWidget.cleanup()
        self._stack.mainChildWidget.deleteLater()
        self._stack.mainChildWidget = None

        gc.collect()
