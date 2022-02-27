# This Python file uses the following encoding: utf-8
import logging
import math
import os
import sys
import threading
import time
from enum import Enum

from python_qt_binding import QtCore, QtWidgets
from python_qt_binding.QtCore import QSize, QFile, Qt, QPoint
from python_qt_binding.QtGui import QPainter, QBrush, QPen
from python_qt_binding.QtWidgets import QPushButton, QMainWindow, QWidget
from ament_index_python import get_resource
from python_qt_binding import loadUi

from shared.utils.utils import initializeRobotsOptions
from shared.inner_communication import innerCommunication

class DeletedRobotScreenWidget(QWidget):
    def __init__(self,node=None, plugin=None, stackWidget=None):
        super(DeletedRobotScreenWidget, self).__init__()

        self.stackWidget=stackWidget

        _, package_path = get_resource('packages', 'shared')
        ui_file = os.path.join(package_path, 'share', 'shared', 'resource', 'deleted_robot.ui')
        loadUi(ui_file, self)

        self.comboBox.addItem('')
        initializeRobotsOptions(self.comboBox)

        innerCommunication.deleteRobotSignal.connect(self.onDeleteRobotSignal)

        self.comboBox.currentIndexChanged.connect(self.onChoosenRobotChange)

    def onChoosenRobotChange(self,event):
        data = self.comboBox.currentData()
        print('onChoosenRobotChange')
        print('onChoosenRobotChange')
        print( self.stackWidget)
        # self.stackWidget.onDeletedRobotScreenReturn(data)


    def onDeleteRobotSignal(self, data):
        indexOfElementToBeRemoved = self.comboBox.findData(data)
        self.comboBox.removeItem(indexOfElementToBeRemoved)