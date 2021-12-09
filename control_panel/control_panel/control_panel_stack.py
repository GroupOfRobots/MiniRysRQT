# This Python file uses the following encoding: utf-8
from python_qt_binding.QtWidgets import QWidget, QStackedWidget,QHBoxLayout,QListWidget,QGridLayout,QLayout

# from elements.button import Button
from .control_panel_widget import ControlPanelWidget
from .setup import SetupWidget


class ControlPanelStack(QWidget):
    def __init__(self, node, plugin=None):
        super(ControlPanelStack, self).__init__()

        self.stack = QStackedWidget(self)

        self.controlPanelWidget = ControlPanelWidget(node, self)
        self.controlPanelWidget3 = SetupWidget(node, self)

        self.stack.addWidget(self.controlPanelWidget)
        self.stack.addWidget(self.controlPanelWidget3)


        hbox = QGridLayout(self)
        hbox.addWidget(self.stack)

        self.stack.setCurrentIndex(0)

        self.setLayout(hbox)