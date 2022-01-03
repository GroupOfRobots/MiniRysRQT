# This Python file uses the following encoding: utf-8
import os

from python_qt_binding.QtWidgets import QPushButton, QWidget,QLineEdit,QTableWidget,QTableWidgetItem
from ament_index_python import get_resource
from python_qt_binding import loadUi

import json



class SetupWidget(QWidget):
    def __init__(self, node, plugin=None):
        super(SetupWidget, self).__init__()

        _, package_path = get_resource('packages', 'setup_panel')
        ui_file = os.path.join(package_path, 'share', 'setup_panel', 'resource', 'setup.ui')
        loadUi(ui_file, self)

        self.backButton.clicked.connect(self.backClicked)

        self.loadJson()

    def backClicked(self):
        parent=self.parent()
        parent.setCurrentIndex(0)

    def resizeEvent(self, event):
        print("resize")
        # print(self.backwardButton.size().width())
        # self.setIconSize()

    def loadJson(self):
        _, package_path = get_resource('packages', 'setup_panel')

        dataFile = os.path.join(package_path, 'share', 'setup_panel', 'data', 'data.json')
        print(dataFile)
        f = open(dataFile)

        # a= new QLineEdit()

        # returns JSON object as
        # a dictionary
        data = json.load(f)
        # data.robotName
        print(data['robotName'])
        # robotName=data['robotName']
        self.robotNameInput.setText(data['robotName'])
        self.forwardKeyInput.setText(data['controlKeys']['forward'])
        self.rightKeyInput.setText(data['controlKeys']['right'])
        self.backwardKeyInput.setText(data['controlKeys']['backward'])
        self.leftKeyInput.setText(data['controlKeys']['left'])

        dynamic= data['dynamic']
        forwardDynamic=dynamic['forward']
        forwardLeftDynamic=dynamic['forwardLeft']
        forwardRightDynamic=dynamic['forwardRight']
        backwardDynamic=dynamic['backward']
        self.dynamicTable.setItem(0,0,QTableWidgetItem(str(forwardDynamic['leftEngine'])))
        self.dynamicTable.setItem(0,1,QTableWidgetItem(forwardDynamic['rightEngine']))
        self.dynamicTable.setItem(0,2,QTableWidgetItem(forwardDynamic['inertia']))
        self.dynamicTable.setItem(1,2,QTableWidgetItem('aaaaa'))
        self.dynamicTable.setItem(1,3,QTableWidgetItem(44))
        # a= new QTableWidget()
        # f = open('data.json')
        # json.load(dataFile)

        print(forwardDynamic)
        print(forwardDynamic['leftEngine'])
        print(str(forwardDynamic['leftEngine']))
