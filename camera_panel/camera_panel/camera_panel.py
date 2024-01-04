from shared.base_plugin.base_plugin import BasePlugin

from .camera_panel_stack import CameraPanelStack


class CameraPanel(BasePlugin):
    def __init__(self, context):
        super(CameraPanel, self).__init__(context)
        self.context = context
        self.setObjectName('CameraPanel')

        stack = CameraPanelStack(context.node)
        self.setStackWidget(stack, 'Camera Panel')
