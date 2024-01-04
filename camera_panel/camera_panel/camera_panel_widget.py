# This Python file uses the following encoding: utf-8
from datetime import datetime

import requests
from python_qt_binding.QtCore import pyqtSlot, Qt
from python_qt_binding.QtGui import QPixmap, QImage
from python_qt_binding.QtWidgets import QFileDialog
from shared.alert.alert import Alert
from shared.base_widget.base_widget import BaseWidget
from shared.enums import PackageNameEnum

from .threads.camera_connection_thread import CameraConnectionThread


class CameraPanelWidget(BaseWidget):
    def __init__(self, stack=None, node=None):
        super(CameraPanelWidget, self).__init__(stack, PackageNameEnum.CameraPanel, node=node)
        self.setRobotOnScreen()

        self.initCameraConnectionThread()

        self.screenshotButtonUI.clicked.connect(self.captureScreenshot)

        self.imageWidthSliderUI.sliderReleased.connect(self.sliderSetImageWidth)
        self.imageHeightSliderUI.sliderReleased.connect(self.sliderSetImageHeight)

        self.imageWidthSpinBoxUI.valueChanged.connect(self.spinBoxSetImageWidth)
        self.imageHeightSpinBoxUI.valueChanged.connect(self.spinBoxSetImageHeight)

    def initializeRobotSettings(self):
        self.host = getSSHHost(self.data)
        if self.host is None or self.host == '' and self.firstInitialization:
            self.firstInitialization = False
            Alert(self,self.displayName, "Host was not defined")

        try:
            self.setCameraConnectionThreadData()
        except AttributeError:
            pass

    def initCameraConnectionThread(self):
        self.cameraConnectionThread = CameraConnectionThread(url=self.getRequestUrl(),
                                                             displayFramerateUI=self.displayFramerateUI,
                                                             screenshotButtonUI=self.screenshotButtonUI,
                                                             counterLabelUI=self.counterLabelUI,
                                                             imageWidth=int(self.imageWidthSliderUI.value()),
                                                             imageHeight=int(self.imageHeightSliderUI.value()))

        self.cameraConnectionThread.changePixmap.connect(self.displayImage)
        self.cameraConnectionThread.start()

    def setCameraConnectionThreadData(self):
        self.cameraConnectionThread.url = self.getRequestUrl()
        self.cameraConnectionThread.host = self.host

    def getRequestUrl(self):
        return 'http://' + self.host + ':8000/stream.mjpg'

    def getHeightRange(self):
        heightMaximum = self.imageHeightSpinBoxUI.maximum()
        heightMinimum = self.imageHeightSpinBoxUI.minimum()

        return heightMaximum, heightMinimum, (heightMaximum - heightMinimum)

    def getWidthRange(self):
        widthMaximum = self.imageWidthSpinBoxUI.maximum()
        widthMinimum = self.imageWidthSpinBoxUI.minimum()

        return widthMaximum, widthMinimum, (widthMaximum - widthMinimum)

    def widthAspectRatio(self, height):
        if self.aspectRatioCheckBoxUI.isChecked():
            heightMaximum, heightMinimum, heightRange = self.getHeightRange()

            aspectRatio = (height - heightMinimum) / heightRange

            widthMaximum, widthMinimum, widthRange = self.getWidthRange()

            width = int(aspectRatio * widthRange + widthMinimum)
            self.imageWidthSpinBoxUI.setValue(width)
            self.imageWidthSliderUI.setValue(width)

    def heightAspectRatio(self, width):
        if self.aspectRatioCheckBoxUI.isChecked():
            widthMaximum, widthMinimum, widthRange = self.getWidthRange()

            aspectRatio = (width - widthMinimum) / widthRange

            heightMaximum, heightMinimum, heightRange = self.getHeightRange()

            height = int(aspectRatio * heightRange + heightMinimum)
            self.imageHeightSpinBoxUI.setValue(height)
            self.imageHeightSliderUI.setValue(height)

    def sliderSetImageWidth(self):
        width = int(self.imageWidthSliderUI.value())
        self.imageWidthSpinBoxUI.setValue(width)
        self.cameraConnectionThread.setFrameWidth(width)

        self.heightAspectRatio(width)

    def sliderSetImageHeight(self):
        height = int(self.imageHeightSliderUI.value())
        self.imageHeightSpinBoxUI.setValue(height)
        self.cameraConnectionThread.setFrameHeight(height)

        self.widthAspectRatio(height)

    def spinBoxSetImageWidth(self):
        if not self.imageWidthSpinBoxUI.hasFocus():
            return
        width = int(self.imageWidthSpinBoxUI.value())
        self.imageWidthSliderUI.setValue(width)
        self.cameraConnectionThread.setFrameWidth(width)

        self.heightAspectRatio(width)

    def spinBoxSetImageHeight(self):
        if not self.imageHeightSpinBoxUI.hasFocus():
            return
        height = int(self.imageHeightSpinBoxUI.value())
        self.imageHeightSliderUI.setValue(height)
        self.cameraConnectionThread.setFrameHeight(height)

        self.widthAspectRatio(height)

    def captureScreenshot(self):
        try:
            url = 'http://' + self.host + ':8000/stream.mjpg'
            jsonBody = {'width': self.imageWidthSliderUI.value(), 'height': self.imageHeightSliderUI.value()}
            contents = requests.post(url, json=jsonBody, timeout=2.50)
            image = QImage()

            image.loadFromData(contents.content)
            options = QFileDialog.Options()
            options |= QFileDialog.ReadOnly
            now = datetime.now()

            fileName = now.strftime("%Y-%m-%d-%H-%M-%S") + '-minirys-camera.png'

            fileName, _ = QFileDialog.getSaveFileName(self, "QFileDialog.getSaveFileName()", fileName,
                                                      "PNG Files (*.png);;JPG Files (*.jpg);;All Files (*)",
                                                      options=options)
            if fileName:
                image.save(fileName)
        except Exception as exception:
            print(exception)
            pass

    def cleanup(self):
        self.cameraConnectionThread.terminate()

    def restoreFunctionalities(self):
        self.cameraConnectionThread.resetCounters()
        self.cameraConnectionThread.start()

    @pyqtSlot(QImage)
    def displayImage(self, image):
        imageWidth = self.cameraStreamDisplayUI.width()
        imageHeight = self.cameraStreamDisplayUI.height()
        image = image.scaled(imageWidth, imageHeight, Qt.IgnoreAspectRatio)

        pixmap = QPixmap(image)
        self.cameraStreamDisplayUI.setPixmap(pixmap)
