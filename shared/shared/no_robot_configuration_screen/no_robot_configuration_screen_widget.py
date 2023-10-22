# This Python file uses the following encoding: utf-8
import os

from shared.base_widget.base_widget import BaseWidget
from shared.enums import PackageNameEnum
from python_qt_binding.QtWidgets import QWidget
from ament_index_python import get_resource
from python_qt_binding import loadUi
from shared.inner_communication import innerCommunication


class NoRobotConfigurationScreenWidget(QWidget):
    def __init__(self, stack=None, node=None):
        super(NoRobotConfigurationScreenWidget, self).__init__()
        self.stack=stack
        self.loadUI()
        innerCommunication.addRobotSignal.connect(self.onAddRobotSignal)

    def loadUI(self):
            _, packagePath = get_resource('packages', 'shared')
            uiFile = os.path.join(packagePath, 'share', 'shared', 'resource',
                                 'no_robot_configuration.ui')
            loadUi(uiFile, self)


    def onAddRobotSignal(self):
        self.stack.addFirstRobot()