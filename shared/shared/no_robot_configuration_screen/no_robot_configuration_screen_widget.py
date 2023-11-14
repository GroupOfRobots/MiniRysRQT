# This Python file uses the following encoding: utf-8
import os

from ament_index_python import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from shared.inner_communication import innerCommunication
from shared.utils.load_ui_file import loadUiFile

from python_qt_binding.QtWidgets import QWidget

from shared.enums import PackageNameEnum, packageNameToUIFileMap

class NoRobotConfigurationScreenWidget(QWidget):
    def __init__(self, stack=None, node=None):
        super(NoRobotConfigurationScreenWidget, self).__init__()
        self.stack = stack
        loadUiFile(self, 'shared', 'no_robot_configuration.ui')
        innerCommunication.addRobotSignal.connect(self.onAddRobotSignal)

    def onAddRobotSignal(self):
        self.stack.addFirstRobot()
