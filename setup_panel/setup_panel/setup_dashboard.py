# This Python file uses the following encoding: utf-8
import os, os.path

import can
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import QPushButton, QWidget,QLineEdit,QTableWidget, QHBoxLayout,QGroupBox,QFormLayout,QLabel,QComboBox
from ament_index_python import get_resource
from python_qt_binding import loadUi

# from PyQt5.QtWidgets import QVBoxLayout

from .dashboard_element import DashboardElementWidget
# from .setup_dashboard_stack import SetupDashboardStackWidget


class SetupDashboardWidget(QWidget):
    def __init__(self, node, plugin=None, context=None):
        super(SetupDashboardWidget, self).__init__()

        self.setupDashboardStackWidget = plugin

        _, shared_path = get_resource('packages', 'shared')
        _, package_path = get_resource('packages', 'setup_panel')
        ui_file = os.path.join(package_path, 'share', 'setup_panel', 'resource', 'setup_dashboard.ui')
        loadUi(ui_file, self)


        self.mygroupbox = QGroupBox()
        self.myForm = QFormLayout()
        labellist = []
        combolist = []

        dataFilePath = os.path.join(shared_path, 'share', 'shared', 'data', 'robots')

        for index,fileName in enumerate(os.listdir(dataFilePath)):
            combolist.append(DashboardElementWidget(self,fileName=fileName,context=context))
            self.myForm.addRow(combolist[index])

        # myform.setLayout(Qt.AlignTop)
        self.mygroupbox.setLayout(self.myForm)
        # mygroupbox.setAlignment(Qt.AlignTop)

        # mygroupbox.se
        # self.robotsGroupBox.setLayout(myform)

        # self.scrollArea.setAlignment(Qt.AlignTop)
        # self.scrollArea.setLayout(myform)
        self.scrollArea.setWidget(self.mygroupbox)
        # self.scrollArea.setWidget(self.robotsGroupBox)

        # self.scrollArea.setWidgetResizable(True)
        # self.scrollArea.setFixedHeight(100)

        self.addNewRobotButton.clicked.connect(self.addNewRobot)

    def addNewRobot(self):
        self.setupDashboardStackWidget.goToSettings()


