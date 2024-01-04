from minirys_msgs.srv import RecordVideoStop
from python_qt_binding.QtCore import QThread, pyqtSignal
from shared.alert.alert import Alert


class StopRecordingThread(QThread):
    stopRecordingResponse = pyqtSignal(object)

    def __init__(self, cameraVideoRecorderPanelWidget):
        super(QThread, self).__init__()
        self.cameraVideoRecorderPanelWidget = cameraVideoRecorderPanelWidget
        self.recordButtonUI = cameraVideoRecorderPanelWidget.recordButtonUI
        self.recordVideoStopService = cameraVideoRecorderPanelWidget.recordVideoStopService

    def run(self):
        self.recordButtonUI.setEnabled(False)
        while not self.recordVideoStopService.wait_for_service(timeout_sec=2.0):
            Alert(self.cameraVideoRecorderPanelWidget,self.cameraVideoRecorderPanelWidget.displayName, "Stop recording service not available")
            return

        req = RecordVideoStop.Request()
        response = self.recordVideoStopService.call(req)

        self.stopRecordingResponse.emit(response)
