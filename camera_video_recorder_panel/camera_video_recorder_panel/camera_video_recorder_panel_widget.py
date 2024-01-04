# This Python file uses the following encoding: utf-8
from minirys_msgs.srv import RecordVideoStart, RecordVideoStop
from python_qt_binding.QtWidgets import QFileDialog
from shared.alert.alert import Alert
from shared.base_widget.base_widget import BaseWidget
from shared.enums import PackageNameEnum
from shared.utils.ssh_data import getSSHData

from .elements.fetch_recording_spinner import FetchRecordingSpinner
from .elements.recording_spinner import RecordingSpinner
from .threads.fetch_file_thread import FetchFileThread
from .threads.start_recording_thread import StartRecordingThread
from .threads.stop_recording_thread import StopRecordingThread


class CameraVideoRecorderPanelWidget(BaseWidget):
    def __init__(self, stack=None, node=None):
        super(CameraVideoRecorderPanelWidget, self).__init__(stack, PackageNameEnum.CameraVideoRecorderPanel, node=node)
        self.setRobotOnScreen()

        self.recordButtonUI.clicked.connect(self.onRecordButtonClicked)
        self.fileSizeWrapperWidgetUI.setVisible(False);

        self.isRecording = False

        self.recordingSpinner = RecordingSpinner(self.widgetUI)

        self.fetchRecordingSpinner = FetchRecordingSpinner(self.widgetUI)

        self.recordVideoStartService = self.node.create_client(RecordVideoStart, 'start_video_recording')
        self.recordVideoStopService = self.node.create_client(RecordVideoStop, 'stop_video_recording')

    def onRecordButtonClicked(self):
        if not self.isRecording:
            self.startRecordingThread = StartRecordingThread(self)
            self.startRecordingThread.startRecordingResponse.connect(self.startRecording)
            self.startRecordingThread.start()
        else:
            self.stopRecordingThread = StopRecordingThread(self)
            self.stopRecordingThread.stopRecordingResponse.connect(self.stopRecording)
            self.stopRecordingThread.start()

    def startRecording(self, response):
        self.recordButtonUI.setText("STOP RECORDING")
        self.recordingSpinner.start()
        self.isRecording = True

    def stopRecording(self, response):
        self.isRecording = False
        remoteVideoFilePath = response.video_file_path

        self.recordingSpinner.stop()

        self.filePath = self.getVideoFilePath()

        if self.filePath == '':
            self.recordButtonUI.setText("START RECORDING")
            self.recordButtonUI.setEnabled(True)
            Alert(self, self.displayName, "You canceled action hence video will not be fetched")
            return

        self.recordButtonUI.setText("FETCHING FILE")
        self.recordButtonUI.setEnabled(False)

        self.fetchFile(remoteVideoFilePath)

    def fetchFile(self, remoteVideoFilePath):
        self.fileSizeWrapperWidgetUI.setVisible(True)
        self.fetchRecordingSpinner.start()
        self.fetchFileThread = FetchFileThread(filePath=self.filePath,
                                               remoteVideoFilePath=remoteVideoFilePath,
                                               host=self.host,
                                               port=self.port,
                                               username=self.username,
                                               password=self.password,
                                               fileSizeLabelUI=self.fileSizeLabelUI,
                                               )
        self.fetchFileThread.finished.connect(self.onFetchFileThreadFinished)
        self.fetchFileThread.start()

    def onFetchFileThreadFinished(self):
        self.fetchRecordingSpinner.stop()
        self.recordButtonUI.setText("START RECORDING")
        self.recordButtonUI.setEnabled(True)
        self.fileSizeWrapperWidgetUI.setVisible(False)

    def getVideoFilePath(self):
        options = QFileDialog.Options()
        options |= QFileDialog.ReadOnly
        filePath, _ = QFileDialog.getSaveFileName(self, "QFileDialog.getSaveFileName()",
                                                  self.output,
                                                  "MP4 Files (*.mp4);;All Files (*)",
                                                  options=options)
        return filePath

    def initializeRobotSettings(self):
        self.host, self.username, self.password, self.port = getSSHData(self.data)

        videoRecorder = self.data.get("videoRecorder", {})
        self.output = videoRecorder.get("output", None)
        self.quality = videoRecorder.get("quality", None)
        config = videoRecorder.get("config", {})
        self.width = config.get("width", 640)
        self.height = config.get("height", 480)

        if self.host is None or self.host == '':
            exceptionToDisplay = "SSH host was not defined"
            Alert(self, self.displayName, exceptionToDisplay)

        self.displayReecordingData()

    def displayReecordingData(self):
        self.widthLabelUI.setText(str(self.width))
        self.heightLabelUI.setText(str(self.height))
        self.outputLabelUI.setText(self.output)
        self.qualityLabelUI.setText(self.quality)

    def cleanup(self):
        if self.isRecording:
            req = RecordVideoStop.Request()
            return self.recordVideoStopService.call(req)
