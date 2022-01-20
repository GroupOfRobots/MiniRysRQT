# This Python file uses the following encoding: utf-8
from python_qt_binding.QtWidgets import QWidget, QStackedWidget,QHBoxLayout,QListWidget,QGridLayout,QLayout

from setup_panel.setup import SetupWidget
from .control_panel_widget import ControlPanelWidget

class ControlPanelStack(QWidget):
    def __init__(self, node, plugin=None):
        super(ControlPanelStack, self).__init__()

        self.node = node

        self.stack = QStackedWidget(self)


        self.controlPanelWidget = ControlPanelWidget(node, plugin=self)
        self.setupWidget = SetupWidget(node,plugin= self)

        self.stack.addWidget(self.controlPanelWidget)
        # self.stack.addWidget(self.setupWidget)

        hbox = QGridLayout(self)
        hbox.addWidget(self.stack)

        self.stack.setCurrentIndex(0)

        self.setLayout(hbox)

    def goToSettings(self, fileName=None):
        if hasattr(self, 'setupWidget'):
            self.stack.removeWidget(self.setupWidget)
        self.setupWidget = SetupWidget(self.node, fileName=fileName)
        self.stack.addWidget(self.setupWidget)
        self.stack.setCurrentIndex(1)