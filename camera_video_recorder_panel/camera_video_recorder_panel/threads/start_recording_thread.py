from minirys_msgs.srv import RecordVideoStart
from python_qt_binding.QtCore import QThread, pyqtSignal
from shared.inner_communication import innerCommunication


class StartRecordingThread(QThread):
    startRecordingResponse = pyqtSignal(object)

    def __init__(self, cameraVideoRecorderPanelWidget):
        super(QThread, self).__init__()
        self.cameraVideoRecorderPanelWidget = cameraVideoRecorderPanelWidget
        self.recordButtonUI = cameraVideoRecorderPanelWidget.recordButtonUI
        self.recordVideoStartService = cameraVideoRecorderPanelWidget.recordVideoStartService

    def run(self):
        self.recordButtonUI.setEnabled(False)
        while not self.recordVideoStartService.wait_for_service(timeout_sec=2.0):
            self.recordButtonUI.setEnabled(True)
            alertData = {
                "widgetName": self.cameraVideoRecorderPanelWidget.displayName,
                "alertText": "Recording service not available"
            }
            innerCommunication.showAlert.emit(alertData)
            return
        self.recordButtonUI.setEnabled(True)

        req = RecordVideoStart.Request()
        req.width = self.cameraVideoRecorderPanelWidget.width
        req.height = self.cameraVideoRecorderPanelWidget.height
        req.quality = self.cameraVideoRecorderPanelWidget.quality
        response = self.recordVideoStartService.call(req)

        self.startRecordingResponse.emit(response)
