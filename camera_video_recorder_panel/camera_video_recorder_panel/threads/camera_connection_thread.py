import requests
from python_qt_binding.QtCore import QThread
from python_qt_binding.QtGui import QImage
from shared.alert.alert import Alert


class CameraConnectionThread(QThread):
    def __init__(self, url, displayName):
        super(QThread, self).__init__()
        self.displayName = displayName
        self.url = url

    def connectedToService(self):
        image = QImage()
        try:
            jsonBody = {'width': self.imageWidth, 'height': self.imageHeight}
            contents = requests.post(self.url, json=jsonBody, timeout=2.50)
            image.loadFromData(contents.content)

        except Exception as exception:
            exceptionToDisplay = "Camera recorder: " + str(exception)
            Alert(self.displayName, exceptionToDisplay)
            pass

    def run(self):
        try:
            requests.post(self.url, timeout=2.50)
            self.connectedToService()
        except:
            pass
