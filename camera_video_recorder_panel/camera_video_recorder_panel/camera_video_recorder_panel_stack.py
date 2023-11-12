# This Python file uses the following encoding: utf-8

from shared.stack_widget.stack_widget import StackWidget

from .camera_video_recorder_panel_widget import CameraVideoRecorderPanelWidget


class CameraVideoRecorderPanelStack(StackWidget):
    def __init__(self, node=None):
        super(CameraVideoRecorderPanelStack, self).__init__(node=node, constructor=CameraVideoRecorderPanelWidget)
