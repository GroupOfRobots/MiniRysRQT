import gc

from rqt_gui_py.plugin import Plugin


class BasePlugin(Plugin):
    def __init__(self, context):
        super(BasePlugin, self).__init__(context)
        self.context = context

    def setStackWidget(self, stackWidget, title):
        self._stack = stackWidget
        self._stack.setWindowTitle(title)

        self.setWidgetSerialNumber(self._stack)

        self.context.add_widget(self._stack)

    def shutdown_plugin(self):
        self._stack.stack.removeWidget(self._stack.mainChildWidget)
        self._stack.mainChildWidget.cleanup()
        self._stack.mainChildWidget.deleteLater()
        self._stack.mainChildWidget = None

        gc.collect()

    def setWidgetSerialNumber(self, widget):
        if self.context.serial_number() > 1:
            widget.setWindowTitle(
                widget.windowTitle() + (' (%d)' % self.context.serial_number()))