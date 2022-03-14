# This Python file uses the following encoding: utf-8
import os

from ament_index_python import get_resource
from python_qt_binding import loadUi
from shared.base_widget.base_widget import BaseWidget

class DeletedRobotScreenWidget(BaseWidget):
    def __init__(self, stack=None):
        super(DeletedRobotScreenWidget, self).__init__()
        BaseWidget.__init__(self, stack)

        _, packagePath = get_resource('packages', 'shared')
        uiFile = os.path.join(packagePath, 'share', 'shared', 'resource', 'deleted_robot.ui')
        loadUi(uiFile, self)

        self.comboBox.addItem('')
        self.initializeRobotsOptions()

        self.comboBox.currentIndexChanged.connect(self.onChoosenRobotChange)

    def onChoosenRobotChange(self, event):
        data = self.comboBox.currentData()
        self.stack.onDeletedRobotScreenReturn(data)

    def onDeleteRobotSignal(self, data):
        indexOfElementToBeRemoved = self.comboBox.findData(data)
        self.comboBox.removeItem(indexOfElementToBeRemoved)
