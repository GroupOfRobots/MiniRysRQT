# This Python file uses the following encoding: utf-8
import os, os.path

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import QPushButton, QWidget,QLineEdit,QTableWidget,QHBoxLayout,QGroupBox,QFormLayout,QLabel,QComboBox
from ament_index_python import get_resource
from python_qt_binding import loadUi

# from PyQt5.QtWidgets import QVBoxLayout

from .dashboard_element import DashboardElementWidget

class SetupDashboardWidget(QWidget):
    def __init__(self, node, plugin=None):
        super(SetupDashboardWidget, self).__init__()

        print('SetupDashboardWidget')
        print(self)
        print(node)
        # node.goToSettings('aa')

        _, package_path = get_resource('packages', 'setup_panel')
        ui_file = os.path.join(package_path, 'share', 'setup_panel', 'resource', 'setup_dashboard.ui')
        loadUi(ui_file, self)


        self.mygroupbox = QGroupBox()
        self.myForm = QFormLayout()
        labellist = []
        combolist = []

        dataFilePath = os.path.join(package_path, 'share', 'setup_panel', 'data', 'robots')

        for index,fileName in enumerate(os.listdir(dataFilePath)):
            combolist.append(DashboardElementWidget(self,fileName=fileName))
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

    # def resizeEvent(self, event):
    #     print(self.scrollAreaWrapper)
    #     print(self.scrollAreaWrapper.geometry())
    #     print(self.scrollAreaWrapper.geometry().height())
    #     print(self.scrollAreaWrapper.geometry().height())
    #     self.scrollArea.setFixedHeight(self.scrollAreaWrapper.geometry().height())
    #     self.update()

    def addNewRobot(self):
        print('aaaaaaaaaaaaaaaaaaaaaa')
        print(self.parentWidget())
        print(self.parent())
        print(self.parent().parent())
        print(self.parent().parent().parent())
        # print(self.parent().parent().parent().parent().parent().parent())
        self.parent().parent().goToSettings()


