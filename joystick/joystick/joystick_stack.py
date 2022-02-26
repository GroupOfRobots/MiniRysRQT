# This Python file uses the following encoding: utf-8

from python_qt_binding.QtWidgets import QWidget, QStackedWidget, QStackedLayout
from shared.stack_widget.stack_widget import StackWidget

from .joystick_widget import JoystickWidget


class JoystickStack(StackWidget):
    def __init__(self, node=None, plugin=None):
        super(JoystickStack, self).__init__()
        # self.stack = QStackedWidget(self)

        self.mainChildWidget = JoystickWidget(node=node, plugin=self, stack=self)
        self.stack.addWidget(self.mainChildWidget)

    #     layout = QStackedLayout(self)
    #     layout.addWidget(self.stack)
    #
    #     self.setLayout(layout)
    #
    # def goToDeletedRobotScreen(self):
    #     self.deletedRobotScreenWidget = DeletedRobotScreenWidget(stackWidget=self)
    #     self.stack.addWidget(self.deletedRobotScreenWidget)
    #     self.stack.setCurrentWidget(self.deletedRobotScreenWidget)
