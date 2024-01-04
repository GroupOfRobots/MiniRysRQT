from shared.base_plugin.base_plugin import BasePlugin

from .camera_video_recorder_panel_stack import CameraVideoRecorderPanelStack


class CameraVideoRecorderPanel(BasePlugin):
    def __init__(self, context):
        super(CameraVideoRecorderPanel, self).__init__(context)
        self.setObjectName('CameraVideoRecorderPanel')

        stack = CameraVideoRecorderPanelStack(node=context.node)
        self.setStackWidget(stack, 'Camera Video Recorder Panel')
