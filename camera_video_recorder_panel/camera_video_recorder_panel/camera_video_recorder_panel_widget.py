# This Python file uses the following encoding: utf-8

from shared.alert.alert import Alert
from shared.base_widget.base_widget import BaseWidget
from shared.enums import PackageNameEnum

from .threads.camera_connection_thread import CameraConnectionThread


class CameraVideoRecorderPanelWidget(BaseWidget):
    def __init__(self, stack=None, node=None):
        super(CameraVideoRecorderPanelWidget, self).__init__(stack, PackageNameEnum.CameraVideoRecorderPanel, node=node)

        self.setRobotOnScreen()
        self.settingsButtonUI.clicked.connect(self.settingsClicked)

        self.cameraConnectionThread = CameraConnectionThread(url=self.getRequestUrl(), displayName=self.displayName)

        self.cameraConnectionThread.start()
        self.recordButtonUI.clicked.connect(self.onRecordButtonClicked)

    def onRecordButtonClicked(self):
        print("onRecordButtonClicked")

    def initializeRobotSettings(self):
        sshData = self.data.get('ssh', {})
        self.host = sshData.get('host')

        if self.host is None or self.host == '':
            exceptionToDisplay = "SSH host was not defined"
            Alert(self.displayName, exceptionToDisplay)
        try:
            if self.cameraConnectionThread is not None:
                self.cameraConnectionThread.url = self.getRequestUrl()
                self.cameraConnectionThread.host = self.host
        except AttributeError:
            pass

    def getRequestUrl(self):
        return 'http://' + self.host + ':8000/record'

    def onShtudownPlugin(self):
        self.cameraConnectionThread.terminate()

    def settingsClicked(self):
        self.stack.goToSettings(self.dataFilePath)
