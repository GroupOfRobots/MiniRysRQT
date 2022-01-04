# This Python file uses the following encoding: utf-8
import os

from python_qt_binding.QtWidgets import QPushButton, QWidget,QLineEdit,QTableWidget,QTableWidgetItem
from ament_index_python import get_resource
from python_qt_binding import loadUi

class DashboardElementWidget(QWidget):
    def __init__(self, node, plugin=None):
        super(DashboardElementWidget, self).__init__()

        _, package_path = get_resource('packages', 'setup_panel')
        ui_file = os.path.join(package_path, 'share', 'setup_panel', 'resource', 'dashboard_element.ui')
        loadUi(ui_file, self)