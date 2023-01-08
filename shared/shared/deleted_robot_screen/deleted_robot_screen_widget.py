# This Python file uses the following encoding: utf-8
from shared.base_widget.base_widget import BaseWidget
from shared.enums import PackageNameEnum


class DeletedRobotScreenWidget(BaseWidget):
    def __init__(self, stack=None):
        super(DeletedRobotScreenWidget, self).__init__(stack, PackageNameEnum.DeletedRobotScreenWidget)
        self.comboBox.currentIndexChanged.disconnect()

        self.stack = stack
        self.comboBox.insertItem(0, '')
        self.comboBox.setCurrentIndex(0)

        self.comboBox.currentIndexChanged.connect(self.stack.onDeletedRobotScreenReturn)