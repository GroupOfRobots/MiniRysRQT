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

        self.dataFilePath = os.path.join(package_path, 'share', 'setup_panel', 'data', 'data.json')

        self.backButton.clicked.connect(self.backClicked)
        self.saveButton.clicked.connect(self.saveClicked)

        self.loadJson()

    def backClicked(self):
        parent=self.parent()
        parent.setCurrentIndex(0)

    def saveClicked(self):
        dataFile = open(self.dataFilePath,'r')
        data = json.load(dataFile)
        dataFile.close()
        dataFile = open(self.dataFilePath, 'w')
        data['controlKeys']['forward']=self.forwardKeyInput.text()
        json.dump(data,dataFile)
        dataFile.close()

    def resizeEvent(self, event):
        print("resize")


    def loadJson(self):
        dataFile = open(self.dataFilePath)
        data = json.load(dataFile)
        dataFile.close()

        self.robotNameInput.setText(data['robotName'])

        # CONTROL KEYS
        controlKeys=data['controlKeys']
        self.forwardKeyInput.setText(controlKeys['forward'])
        self.rightKeyInput.setText(controlKeys['right'])
        self.backwardKeyInput.setText(controlKeys['backward'])
        self.leftKeyInput.setText(controlKeys['left'])

        # DYNAMIC
        dynamic= data['dynamic']
        forwardDynamic=dynamic['forward']
        forwardLeftDynamic=dynamic['forwardLeft']
        forwardRightDynamic=dynamic['forwardRight']

        backwardDynamic=dynamic['backward']
        backwardLeftDynamic=dynamic['backward']
        backwardRightDynamic=dynamic['backward']

        # FORWARD
        self.dynamicTable.setItem(0,0,QTableWidgetItem(str(forwardDynamic['leftEngine'])))
        self.dynamicTable.setItem(0,1,QTableWidgetItem(str(forwardDynamic['rightEngine'])))
        self.dynamicTable.setItem(0,2,QTableWidgetItem(str(forwardDynamic['inertia'])))

        self.dynamicTable.setItem(1,0,QTableWidgetItem(str(forwardLeftDynamic['leftEngine'])))
        self.dynamicTable.setItem(1,1,QTableWidgetItem(str(forwardLeftDynamic['rightEngine'])))
        self.dynamicTable.setItem(1,2,QTableWidgetItem(str(forwardLeftDynamic['inertia'])))

        self.dynamicTable.setItem(2,0,QTableWidgetItem(str(forwardRightDynamic['leftEngine'])))
        self.dynamicTable.setItem(2,1,QTableWidgetItem(str(forwardRightDynamic['rightEngine'])))
        self.dynamicTable.setItem(2,2,QTableWidgetItem(str(forwardRightDynamic['inertia'])))

        # BACKWARD
        self.dynamicTable.setItem(3, 0, QTableWidgetItem(str(backwardDynamic['leftEngine'])))
        self.dynamicTable.setItem(3, 1, QTableWidgetItem(str(backwardDynamic['rightEngine'])))
        self.dynamicTable.setItem(3, 2, QTableWidgetItem(str(backwardDynamic['inertia'])))

        self.dynamicTable.setItem(4, 0, QTableWidgetItem(str(backwardLeftDynamic['leftEngine'])))
        self.dynamicTable.setItem(4, 1, QTableWidgetItem(str(backwardLeftDynamic['rightEngine'])))
        self.dynamicTable.setItem(4, 2, QTableWidgetItem(str(backwardLeftDynamic['inertia'])))

        self.dynamicTable.setItem(5, 0, QTableWidgetItem(str(backwardRightDynamic['leftEngine'])))
        self.dynamicTable.setItem(5, 1, QTableWidgetItem(str(backwardRightDynamic['rightEngine'])))
        self.dynamicTable.setItem(5, 2, QTableWidgetItem(str(backwardRightDynamic['inertia'])))

        # JOYSTICK
        joystick = data['joystick']

        joystickForward=joystick['forward']
        joystickRight=joystick['right']
        joystickBackward=joystick['backward']
        joystickLeft =joystick['left']

        self.joystickForward.setItem(0,0, QTableWidgetItem(str(joystickForward['leftEngine'])))
        self.joystickForward.setItem(0,1, QTableWidgetItem(str(joystickForward['rightEngine'])))

        self.joystickRight.setItem(0, 0, QTableWidgetItem(str(joystickRight['leftEngine'])))
        self.joystickRight.setItem(0, 1, QTableWidgetItem(str(joystickRight['rightEngine'])))

        self.joystickBackward.setItem(0, 0, QTableWidgetItem(str(joystickBackward['leftEngine'])))
        self.joystickBackward.setItem(0, 1, QTableWidgetItem(str(joystickBackward['rightEngine'])))

        self.joystickLeft.setItem(0, 0, QTableWidgetItem(str(joystickLeft['leftEngine'])))
        self.joystickLeft.setItem(0, 1, QTableWidgetItem(str(joystickLeft['rightEngine'])))