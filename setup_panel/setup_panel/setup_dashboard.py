# This Python file uses the following encoding: utf-8
import os

from python_qt_binding.QtWidgets import QPushButton, QWidget,QLineEdit,QTableWidget,QHBoxLayout,QGroupBox,QFormLayout,QLabel,QComboBox
from ament_index_python import get_resource
from python_qt_binding import loadUi

from .dashboard_element import DashboardElementWidget

class SetupDashboardWidget(QWidget):
    def __init__(self, node, plugin=None):
        super(SetupDashboardWidget, self).__init__()

        _, package_path = get_resource('packages', 'setup_panel')
        ui_file = os.path.join(package_path, 'share', 'setup_panel', 'resource', 'setup_dashboard.ui')
        loadUi(ui_file, self)


        mygroupbox = QGroupBox('this is my groupbox')
        myform = QFormLayout()
        labellist = []
        combolist = []
        for i in range(15):
            combolist.append(DashboardElementWidget(self))
            myform.addRow(combolist[i])
        mygroupbox.setLayout(myform)

        self.scrollArea.setWidget(mygroupbox)

        # self.scrollArea.setWidgetResizable(True)
        self.scrollArea.setFixedHeight(200)

        # self._widget = DashboardElementWidget(self)
        # self._widget1 = DashboardElementWidget(self)
        # self._widget2 = DashboardElementWidget(self)
        # self._widget3 = DashboardElementWidget(self)
        #
        # # QScrollArea
        # self.scrollArea.setWidget(self._widget)
        # self.scrollArea.setWidget(self._widget1)
        # self.scrollArea.setWidget(self._widget2)
        # self.scrollArea.setWidget(self._widget3)
        # self.a= QHBoxLayout()
        # #
        # self.a.addWidget(self._widget)
        # self.a.addWidget(self._widget1)
        # self.a.addWidget(self._widget2)
        # self.scrollArea.setWidget(self.a)

