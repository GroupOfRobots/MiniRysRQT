# This Python file uses the following encoding: utf-8
import json
import os

from python_qt_binding.QtWidgets import QAbstractSpinBox, QFileDialog, QMessageBox
from shared.base_widget.base_widget import BaseWidget

from shared.enums import PackageNameEnum

from python_qt_binding.QtCore import pyqtSignal, QObject, QThread, pyqtSlot, Qt
from python_qt_binding.QtGui import QColor, QFont, QPixmap, QImage

import requests
import time
from datetime import datetime


class Thread(QThread):
    changePixmap = pyqtSignal(QImage)
    counter = 0
    exceptionCounter = 0
    timestamp = 0

    def __init__(self, host, displayFramerateUI, counterLabelUI, imageWidth, imageHeight):
        super(QThread, self).__init__()
        self.url = 'http://' + host + ':8000/stream.mjpg'
        self.displayFramerateUI = displayFramerateUI
        self.counterLabelUI = counterLabelUI

        self.imageWidth = imageWidth
        self.imageHeight = imageHeight

    def connectedToService(self):
        image = QImage()
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
            except:
                self.counter = 0
                self.counterLabelUI.setText('Trying to reconnect: ')
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
            except Exception as e:
                print(e)
                pass

    def run(self):
        try:
            requests.post(self.url, timeout=2.50)
            self.counterLabelUI.setText('Framerate: ')
            self.connectedToService()
        except:
            self.tryingToConnect()

    def setFrameWidth(self, width):
        self.imageWidth = width

    def setFrameHeight(self, height):
        self.imageHeight = height


class AlertThread(QThread):
    def run(self):
        msg = QMessageBox()
        msg.setWindowTitle("Alert")
        msg.setText("This is an alert message")
        msg.setIcon(QMessageBox.Warning)
        msg.exec_()


class CameraPanelWidget(BaseWidget):
    def __init__(self, stack=None, node=None):
        super(CameraPanelWidget, self).__init__(stack, PackageNameEnum.CameraPanel)
        print("CameraPanelWidget")

        self.comboBox.currentIndexChanged.connect(self.setRobotOnScreen)
        self.setRobotOnScreen()
        self.settingsButtonUI.clicked.connect(self.settingsClicked)

        sshData = self.data.get('ssh', {})

        self.host = sshData.get('host')

        if self.host is None or self.host == '':
            self.showAlertThatHostIsNotDefined()

        self.screenshotButtonUI.clicked.connect(self.captureScreenshot)

        self.th = Thread(host=self.host, displayFramerateUI=self.displayFramerateUI, counterLabelUI=self.counterLabelUI,
                         imageWidth=int(self.imageWidthSliderUI.value()),
                         imageHeight=int(self.imageHeightSliderUI.value()))
        self.th.changePixmap.connect(self.displayImage)
        self.th.start()

        self.imageWidthSliderUI.sliderReleased.connect(self.sliderSetImageWidth)
        self.imageHeightSliderUI.sliderReleased.connect(self.sliderSetImageHeight)

        self.imageWidthSpinBoxUI.valueChanged.connect(self.spinBoxSetImageWidth)
        self.imageHeightSpinBoxUI.valueChanged.connect(self.spinBoxSetImageHeight)

        self.aspectRatioCheckBoxUI.stateChanged.connect(self.aspectRatioChanged)

    def aspectRatioChanged(self, event):
        print(event)
        print(self.aspectRatioCheckBoxUI.isChecked())

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
        self.th.setFrameWidth(width)

        self.heightAspectRatio(width)

    def sliderSetImageHeight(self):
        height = int(self.imageHeightSliderUI.value())
        self.imageHeightSpinBoxUI.setValue(height)
        self.th.setFrameHeight(height)

        self.widthAspectRatio(height)

    def spinBoxSetImageWidth(self):
        if not self.imageWidthSpinBoxUI.hasFocus() :
            return
        width = int(self.imageWidthSpinBoxUI.value())
        self.imageWidthSliderUI.setValue(width)
        self.th.setFrameWidth(width)

        self.heightAspectRatio(width)

    def spinBoxSetImageHeight(self):
        print("spinBoxSetImageHeight")
        if not self.imageHeightSpinBoxUI.hasFocus():
            return
        height = int(self.imageHeightSpinBoxUI.value())
        print(height)
        self.imageHeightSliderUI.setValue(height)
        self.th.setFrameHeight(height)

        self.widthAspectRatio(height)

    def showAlertThatHostIsNotDefined(self):
        self.th1 = AlertThread()
        self.th1.start()

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
        except:
            pass

    def onShtudownPlugin(self):
        self.th.terminate()

    def settingsClicked(self):
        self.stack.goToSettings(self.dataFilePath)

    @pyqtSlot(QImage)
    def displayImage(self, image):
        imageWidth = self.cameraStreamDisplayUI.width()
        imageHeight = self.cameraStreamDisplayUI.height()
        image = image.scaled(imageWidth, imageHeight, Qt.IgnoreAspectRatio)

        pixmap = QPixmap(image)
        self.cameraStreamDisplayUI.setPixmap(pixmap)
