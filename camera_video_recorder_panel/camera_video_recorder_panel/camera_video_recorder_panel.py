from rqt_gui_py.plugin import Plugin
from .camera_video_recorder_panel_stack import CameraVideoRecorderPanelStack
from shared.utils.serial_number import setWidgetSerialNumber

class CameraVideoRecorderPanel(Plugin):
    def __init__(self, context):
        super(CameraVideoRecorderPanel, self).__init__(context)
        self.setObjectName('CameraVideoRecorderPanel')

        self._stack = CameraVideoRecorderPanelStack(context.node)
        self._stack.setWindowTitle('Camera Video Recorder Panel')

        setWidgetSerialNumber(context, self._stack)

        context.add_widget(self._stack)

    def shutdown_plugin(self):
        self._stack.mainChildWidget.onShtudownPlugin()

