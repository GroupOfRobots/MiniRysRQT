# This Python file uses the following encoding: utf-8
import json
import os

from python_qt_binding.QtWidgets import QAbstractSpinBox, QFileDialog, QMessageBox
from shared.base_widget.base_widget import BaseWidget

from std_msgs.msg import Float32
from ament_index_python import get_resource
from python_qt_binding import loadUi

from shared.enums import PackageNameEnum

from python_qt_binding.QtCore import pyqtSignal, QObject, QThread, pyqtSlot, Qt
from python_qt_binding.QtGui import QColor, QFont, QPixmap, QImage

import requests
import time


class Thread(QThread):
    changePixmap = pyqtSignal(QImage)
    counter = 0
    exceptionCounter = 0
    timestamp = 0

    def __init__(self, host, displayFramerateUI, counterLabelUI):
        super(QThread, self).__init__()
        self.url = 'http://' + host + ':8000/stream.mjpg'
        self.displayFramerateUI = displayFramerateUI
        self.counterLabelUI = counterLabelUI

    def connectedToService(self):
        image = QImage()
        while True:
            try:
                contents = requests.get(self.url, timeout=2.50)
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
            except:
                pass

    def run(self):
        try:
            requests.get(self.url, timeout=2.50)
            self.counterLabelUI.setText('Framerate: ')
            self.connectedToService()
        except:
            self.tryingToConnect()

class Thread1(QThread):
    def run(self):
        msg = QMessageBox()
        msg.setWindowTitle("Alert")
        msg.setText("This is an alert message")
        msg.setIcon(QMessageBox.Warning)
        msg.exec_()

class CameraPanelWidget(BaseWidget):

    def __init__(self, stack=None, node=None):
        super(CameraPanelWidget, self).__init__(stack, PackageNameEnum.CameraPanel)

        self.comboBox.currentIndexChanged.connect(self.setRobotOnScreen)
        self.setRobotOnScreen()
        self.settingsButtonUI.clicked.connect(self.settingsClicked)

        sshData = self.data.get('ssh', {})
        print(sshData)

        self.host = sshData.get('host')

        if self.host is None or self.host == '':
            self.showAlertThatHostIsNotDefined()


        self.screenshotButtonUI.clicked.connect(self.captureScreenshot)


        self.th = Thread(host=self.host, displayFramerateUI=self.displayFramerateUI, counterLabelUI=self.counterLabelUI)
        self.th.changePixmap.connect(self.displayImage)
        self.th.start()

    def showAlertThatHostIsNotDefined(self):
        self.th1 = Thread1()
        self.th1.start()



    def captureScreenshot(self):
        try:
            url = 'http://' + self.host + ':8000/stream.mjpg'
            contents = requests.get(url, timeout=2.50)
            image = QImage()

            image.loadFromData(contents.content)
            options = QFileDialog.Options()
            options |= QFileDialog.ReadOnly
            fileName, _ = QFileDialog.getSaveFileName(self, "QFileDialog.getSaveFileName()", "",
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
