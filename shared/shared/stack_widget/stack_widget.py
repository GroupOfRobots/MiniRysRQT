# This Python file uses the following encoding: utf-8

from python_qt_binding.QtWidgets import QWidget, QStackedWidget, QStackedLayout
from shared.deleted_robot_screen.deleted_robot_screen import DeletedRobotScreenWidget
from setup_panel.details.setup_widget import SetupWidget

class StackWidget(QWidget):
    def __init__(self):
        super(StackWidget, self).__init__()
        self.stack = QStackedWidget(self)

        layout = QStackedLayout(self)
        layout.addWidget(self.stack)

        self.setLayout(layout)

        self.mainChildWidget=None

    def goToDeletedRobotScreen(self):
        self.deletedRobotScreenWidget = DeletedRobotScreenWidget(stack=self)
        self.stack.addWidget(self.deletedRobotScreenWidget)
        self.stack.setCurrentWidget(self.deletedRobotScreenWidget)

    def onDeletedRobotScreenReturn(self, data):
        self.mainChildWidget.setRobotOnScreen(data)
        self.stack.setCurrentWidget(self.mainChildWidget)
        self.stack.removeWidget(self.deletedRobotScreenWidget)
        self.deletedRobotScreenWidget.deleteLater()
        self.deletedRobotScreenWidget = None

    def goToSettings(self, dataFilePath=None):
        if hasattr(self, 'setupWidget'):
            self.stack.removeWidget(self.setupWidget)
        self.setupWidget = SetupWidget(stack = self, dataFilePath=dataFilePath)
        self.stack.addWidget(self.setupWidget)
        self.stack.setCurrentWidget(self.setupWidget)
