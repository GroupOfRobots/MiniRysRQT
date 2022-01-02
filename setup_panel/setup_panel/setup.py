# This Python file uses the following encoding: utf-8
import os

from python_qt_binding.QtWidgets import QPushButton, QWidget
from ament_index_python import get_resource
from python_qt_binding import loadUi


class SetupWidget(QWidget):
    def __init__(self, node, plugin=None):
        super(SetupWidget, self).__init__()

        _, package_path = get_resource('packages', 'setup_panel')
        ui_file = os.path.join(package_path, 'share', 'setup_panel', 'resource', 'setup.ui')
        loadUi(ui_file, self)

        # self.setFocusPolicy(Qt.ClickFocus)
        # self.setFocus()
        #
        # self.defineButtons()

        # self.show()
        self.backButton.clicked.connect(self.backClicked)

    def backClicked(self):
        parent=self.parent()
        parent.setCurrentIndex(0)

    def resizeEvent(self, event):
        print("resize")
        # print(self.backwardButton.size().width())
        # self.setIconSize()

