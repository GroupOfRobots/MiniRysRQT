# This Python file uses the following encoding: utf-8

from python_qt_binding.QtWidgets import QFileDialog
from shared.alert.alert import Alert
from shared.base_widget.base_widget import BaseWidget
from shared.enums import PackageNameEnum

from .elements.fetch_recording_spinner import FetchRecordingSpinner
from .elements.recording_spinner import RecordingSpinner
from .threads.fetch_file_thread import FetchFileThread


class CameraVideoRecorderPanelWidget(BaseWidget):
    def __init__(self, stack=None, node=None):
        super(CameraVideoRecorderPanelWidget, self).__init__(stack, PackageNameEnum.CameraVideoRecorderPanel, node=node)

        self.setRobotOnScreen()
        self.settingsButtonUI.clicked.connect(self.settingsClicked)

        self.recordButtonUI.clicked.connect(self.onRecordButtonClicked)

        self.isRecording = False

        self.recordingSpinner = RecordingSpinner(self.widgetUI)

        self.fetchRecordingSpinner = FetchRecordingSpinner(self.widgetUI)

    def onRecordButtonClicked(self):
        if not self.isRecording:
            self.startRecording()
        else:
            self.stopRecording()

    def startRecording(self):
        self.recordingSpinner.start()

        url = self.startRecordingRequestUrl()

        jsonBody = {'width': self.width, 'height': self.height, 'output': self.output}

        # requests.post(url, timeout=2.50, json=jsonBody)

        self.isRecording = True
        self.recordButtonUI.setText("RECORDING")

    def stopRecording(self):
        self.isRecording = False
        url = self.stopRecordingRequestUrl()
        # requests.post(url)
        self.recordingSpinner.stop()

        self.filePath = self.getVideoFilePath()

        if self.filePath == '':
            self.recordButtonUI.setText("START RECORDING")
            Alert(self.displayName, "You canceled action hence video will be lost")
            return

        self.recordButtonUI.setText("FETCHING FILE")
        self.recordButtonUI.setEnabled(False)

        self.fetchFile()

    def fetchFile(self):
        self.fetchRecordingSpinner.start()

        self.fetchFileThread = FetchFileThread(filePath=self.filePath,
                                               fileName=self.output,
                                               host=self.host,
                                               port=self.port,
                                               username=self.username,
                                               password=self.password)
        self.fetchFileThread.finished.connect(self.onFetchFileThreadFinished)
        self.fetchFileThread.start()

    def onFetchFileThreadFinished(self):
        self.fetchRecordingSpinner.stop()
        self.recordButtonUI.setText("START RECORDING")
        self.recordButtonUI.setEnabled(True)

    def getVideoFilePath(self):
        options = QFileDialog.Options()
        options |= QFileDialog.ReadOnly
        filePath, _ = QFileDialog.getSaveFileName(self, "QFileDialog.getSaveFileName()",
                                                  self.output,
                                                  "MP4 Files (*.mp4);;All Files (*)",
                                                  options=options)
        return filePath

    def initializeRobotSettings(self):
        sshData = self.data.get('ssh', {})
        self.host = sshData.get('host')
        self.username = sshData.get('username')
        self.password = sshData.get('minirys')
        self.port = sshData.get('port')

        videoRecorder = self.data.get("videoRecorder", {})
        self.output = videoRecorder.get("output", None)
        self.quality = videoRecorder.get("quality", None)
        config = videoRecorder.get("config", {})
        self.width = config.get("width", 640)
        self.height = config.get("height", 480)

        if self.host is None or self.host == '':
            exceptionToDisplay = "SSH host was not defined"
            Alert(self.displayName, exceptionToDisplay)

        self.displayReecordingData()

    def displayReecordingData(self):
        self.widthLabelUI.setText(str(self.width))
        self.heightLabelUI.setText(str(self.height))
        self.outputLabelUI.setText(self.output)
        self.qualityLabelUI.setText(self.quality)

    def startRecordingRequestUrl(self):
        return 'http://' + self.host + ':8000/start-recording'

    def stopRecordingRequestUrl(self):
        return 'http://' + self.host + ':8000/stop-recording'
