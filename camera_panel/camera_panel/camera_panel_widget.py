# This Python file uses the following encoding: utf-8
import json
import os

from python_qt_binding.QtWidgets import QAbstractSpinBox
from shared.base_widget.base_widget import BaseWidget

from std_msgs.msg import Float32
from ament_index_python import get_resource
from python_qt_binding import loadUi

from shared.enums import PackageNameEnum

from python_qt_binding.QtCore import pyqtSignal, QObject, QThread, pyqtSlot, Qt
from python_qt_binding.QtGui import QColor, QFont, QPixmap, QImage

import requests


class Thread(QThread):
    changePixmap = pyqtSignal(QImage)

    def __init__(self, host):
        super(QThread, self).__init__()
        self.url = 'http://' + host + ':8000/stream.mjpg'

    def run(self):
        while True:
            contents = requests.get(self.url, timeout=2.50)
            image = QImage()
            image.loadFromData(contents.content)
            self.changePixmap.emit(image)


class CameraPanelWidget(BaseWidget):

    def __init__(self, stack=None, node=None):
        super(CameraPanelWidget, self).__init__(stack, PackageNameEnum.CameraPanel)

        self.comboBox.currentIndexChanged.connect(self.setRobotOnScreen)
        self.setRobotOnScreen()
        self.settingsButtonUI.clicked.connect(self.settingsClicked)

        sshData = self.data.get('ssh', {})
        print(sshData)

        host = sshData.get('host')
        self.th = Thread(host=host)
        self.th.changePixmap.connect(self.displayImage)
        self.th.start()

    def settingsClicked(self):
        self.stack.goToSettings(self.dataFilePath)

    @pyqtSlot(QImage)
    def displayImage(self, image):
        imageWidth = self.cameraStreamDisplayUI.width()
        imageHeight = self.cameraStreamDisplayUI.height()
        image = image.scaled(imageWidth, imageHeight, Qt.IgnoreAspectRatio)

        pixmap = QPixmap(image)
        self.cameraStreamDisplayUI.setPixmap(pixmap)
