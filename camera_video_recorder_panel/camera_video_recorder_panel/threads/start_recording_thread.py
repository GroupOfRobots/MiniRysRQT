from minirys_msgs.srv import RecordVideoStart
from python_qt_binding.QtCore import QThread, pyqtSignal
from shared.alert.alert import Alert


class StartRecordingThread(QThread):
    startRecordingResponse = pyqtSignal(object)

    def __init__(self, cameraVideoRecorderPanelWidget):
        super(QThread, self).__init__()
        self.cameraVideoRecorderPanelWidget = cameraVideoRecorderPanelWidget
        self.recordButtonUI = cameraVideoRecorderPanelWidget.recordButtonUI
        self.recordVideoStartService = cameraVideoRecorderPanelWidget.recordVideoStartService

    def run(self):
        print("run")
        self.recordButtonUI.setEnabled(False)
        while not self.recordVideoStartService.wait_for_service(timeout_sec=2.0):
            print("here")
            self.recordButtonUI.setEnabled(True)
            itemData = {
                "widgetName": self.cameraVideoRecorderPanelWidget.displayName,
                "alertText": "Recording service not available"
            }
            innerCommunication.showAlert.emit(itemData)
            return
        self.recordButtonUI.setEnabled(True)
        print("wwww")

        req = RecordVideoStart.Request()
        req.width = self.cameraVideoRecorderPanelWidget.width
        req.height = self.cameraVideoRecorderPanelWidget.height
        req.quality = self.cameraVideoRecorderPanelWidget.quality
        response = self.recordVideoStartService.call(req)

        self.startRecordingResponse.emit(response)
