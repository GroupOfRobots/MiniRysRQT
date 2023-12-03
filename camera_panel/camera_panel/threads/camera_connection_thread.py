import time

import requests
from python_qt_binding.QtCore import QThread, pyqtSignal
from python_qt_binding.QtGui import QImage


class CameraConnectionThread(QThread):
    changePixmap = pyqtSignal(QImage)
    counter = 0
    exceptionCounter = 0
    timestamp = 0

    def __init__(self, url, displayFramerateUI,screenshotButtonUI, counterLabelUI, imageWidth, imageHeight):
        super(QThread, self).__init__()
        self.displayFramerateUI = displayFramerateUI
        self.screenshotButtonUI = screenshotButtonUI
        self.counterLabelUI = counterLabelUI
        self.imageWidth = imageWidth
        self.imageHeight = imageHeight
        self.url = url

    def connectedToService(self):
        image = QImage()
        self.screenshotButtonUI.setEnabled(True)
        while True:
            try:
                jsonBody = {'width': self.imageWidth, 'height': self.imageHeight}
                contents = requests.post(self.url, json=jsonBody, timeout=2.50)
                image.loadFromData(contents.content)
                self.changePixmap.emit(image)
                self.counter += 1
                if self.timestamp < time.time() - 1:
                    self.displayFramerateUI.setText(str(self.counter))
                    self.counter = 0
                    self.timestamp = time.time()
            except Exception as exception:
                self.resetCounters()
                self.counterLabelUI.setText('Trying to reconnect: ')
                self.screenshotButtonUI.setEnabled(False)
                self.tryingToConnect()

    def tryingToConnect(self):
        while True:
            self.exceptionCounter += 1
            self.displayFramerateUI.setText(str(self.exceptionCounter))
            time.sleep(1)
            try:
                requests.get(self.url, timeout=2.50)
                self.counterLabelUI.setText('Framerate: ')
                self.exceptionCounter = 0
                self.connectedToService()
                return
            except Exception as exception:
                # print(exception)
                pass

    def run(self):
        try:
            requests.post(self.url, timeout=2.50)
            self.counterLabelUI.setText('Framerate: ')
            self.connectedToService()
        except:
            self.screenshotButtonUI.setEnabled(False)
            self.tryingToConnect()

    def setFrameWidth(self, width):
        self.imageWidth = width

    def setFrameHeight(self, height):
        self.imageHeight = height

    def resetCounters(self):
        self.counter = 0
        self.exceptionCounter = 0
        self.timestamp = 0

        self.displayFramerateUI.setText("")
