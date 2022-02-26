# This Python file uses the following encoding: utf-8

from python_qt_binding.QtWidgets import QWidget, QStackedWidget, QStackedLayout
from shared.deleted_robot_screen.deleted_robot_screen import DeletedRobotScreenWidget


class StackWidget(QWidget):
    def __init__(self, node=None, plugin=None):
        super(StackWidget, self).__init__()
        self.stack = QStackedWidget(self)

        layout = QStackedLayout(self)
        layout.addWidget(self.stack)

        self.setLayout(layout)

        self.mainChildWidget=None

    def goToDeletedRobotScreen(self):
        self.deletedRobotScreenWidget = DeletedRobotScreenWidget(stackWidget=self)
        self.stack.addWidget(self.deletedRobotScreenWidget)
        self.stack.setCurrentWidget(self.deletedRobotScreenWidget)

    def onDeletedRobotScreenReturn(self, data):
        self.mainChildWidget.setRobotOnScreen(data)
        self.stack.setCurrentWidget(self.mainChildWidget)
        self.stack.removeWidget(self.deletedRobotScreenWidget)
        self.deletedRobotScreenWidget.deleteLater()
        self.deletedRobotScreenWidget = None