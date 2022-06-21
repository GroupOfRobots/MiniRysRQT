# This Python file uses the following encoding: utf-8
import json
import os

from ament_index_python import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QTableWidgetItem, QMessageBox
from shared.enums import ControlKeyEnum
from shared.inner_communication import innerCommunication

from .commands.command_element import CommandElementWidget
from .pid.pid_widget import PidWidget
from .ssh_data.ssh_data import SshData


class SetupWidget(QWidget):
    def __init__(self, stack=None, dataFilePath=None):
        super(SetupWidget, self).__init__()

        self.stack = stack

        _, self.sharedPath = get_resource('packages', 'shared')
        _, self.packagePath = get_resource('packages', 'setup_panel')
        ui_file = os.path.join(self.packagePath, 'share', 'setup_panel', 'resource', 'setup.ui')
        loadUi(ui_file, self)

        self.defaultFilePath = os.path.join(self.sharedPath, 'share', 'shared', 'data', 'default.json')

        self.addMode = False

        self.dataFilePath = dataFilePath

        if self.dataFilePath:
            self.data = self.loadData(self.dataFilePath)
        else:
            self.addMode = True
            self.dataPath = os.path.join(self.sharedPath, 'share', 'shared', 'data', 'robots')
            self.data = self.loadData(self.defaultFilePath)

            currentFiles = os.listdir(self.dataPath)
            for index in range(len(os.listdir(self.dataPath)) + 1):
                self.fileName = 'data' + str(index) + '.json'
                if self.fileName in currentFiles:
                    continue
                else:
                    self.dataFilePath = self.dataPath + '/' + self.fileName
                    break

        self.loadJson()

        self.backButton.clicked.connect(self.goBack)
        self.saveButton.clicked.connect(self.saveClicked)

        self.restoreDefaultButton.clicked.connect(self.restoreDefault)

        self.keyInputDictionary = {
            ControlKeyEnum.FORWARD: self.forwardKeyInput,
            ControlKeyEnum.RIGHT: self.rightKeyInput,
            ControlKeyEnum.BACKWARD: self.backwardKeyInput,
            ControlKeyEnum.LEFT: self.leftKeyInput,
            ControlKeyEnum.STABLE: self.stableKeyInput,
        }

        self.addControlKeysValidators()

        self.setupDashboardElements()

        self.sshData = SshData(self)
        self.pidWidget = PidWidget(self)
        self.connectElements()



        self.pidSpeedKpUI.valueChanged.connect(self.test)

    def connectElements(self):
        self.pidAngleKpUI.valueChanged.connect(self.pidWidget.test)
        self.pidAngleKiUI.valueChanged.connect(self.pidWidget.test)
        self.pidAngleKdUI.valueChanged.connect(self.pidWidget.test)

        self.pidSpeedKpUI.valueChanged.connect(self.pidWidget.test)
        self.pidSpeedKiUI.valueChanged.connect(self.pidWidget.test)
        self.pidSpeedKdUI.valueChanged.connect(self.pidWidget.test)

    def test(self, event):
        print("event1233")
        print("event1233")
        print(event)


    def setupDashboardElements(self):
        for i in range(1, 5):
            # print(self.elements)
            element = CommandElementWidget()
            # print(element)
            # self.elements.addWidget(element)
            self.elements.addWidget(element)

    def addControlKeysValidators(self):
        self.keyInputDictionary[ControlKeyEnum.FORWARD].textChanged.connect(
            lambda event: self.validator(event, ControlKeyEnum.FORWARD))
        self.keyInputDictionary[ControlKeyEnum.RIGHT].textChanged.connect(
            lambda event: self.validator(event, ControlKeyEnum.RIGHT))
        self.keyInputDictionary[ControlKeyEnum.BACKWARD].textChanged.connect(
            lambda event: self.validator(event, ControlKeyEnum.BACKWARD))
        self.keyInputDictionary[ControlKeyEnum.LEFT].textChanged.connect(
            lambda event: self.validator(event, ControlKeyEnum.LEFT))
        self.keyInputDictionary[ControlKeyEnum.STABLE].textChanged.connect(
            lambda event: self.validator(event, ControlKeyEnum.STABLE))

    def validator(self, event, key):
        button = self.keyInputDictionary[key]
        newValue = event.upper()
        button.setText(newValue)
        self.controlKeys[key] = newValue

        valuesMap = {}

        for key, value in self.controlKeys.items():
            if value in valuesMap:
                valuesMap[value].append(key)
            else:
                valuesMap[value] = [key]

        disableSaveButton = False
        for _, buttonKeys in valuesMap.items():
            if len(buttonKeys) > 1:
                for inputKey in buttonKeys:
                    color = '#EB5535'  # red
                    self.keyInputDictionary[inputKey].setStyleSheet('QLineEdit { background-color: %s }' % color)
                    disableSaveButton = True
            else:
                self.keyInputDictionary[buttonKeys[0]].setStyleSheet('')

        if disableSaveButton:
            self.saveButton.setEnabled(False)
        else:
            self.saveButton.setEnabled(True)

    def restoreDefault(self):
        reply = QMessageBox.question(self, 'Restore default settings', 'Are you sure to restore default setting?',
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

        if reply == QMessageBox.Yes:
            self.data = self.loadData(self.defaultFilePath)
            self.loadJson()

    def goBack(self):
        self.stack.stack.setCurrentIndex(0)

    def saveClicked(self):
        if self.addMode:
            self.addNewRobot()
            return
        self.updateRobotData()

    def addNewRobot(self):
        self.data['robotName'] = self.robotNameInput.text()
        self.data['controlKeys']['forward'] = self.forwardKeyInput.text()

        currentIds = []
        currentFiles = os.listdir(self.dataPath)
        for file in currentFiles:
            dataFile = open(self.dataPath + '/' + file, 'r')
            data = json.load(dataFile)
            dataFile.close()
            currentIds.append(data['id'])

        id = None
        for possibleId in range(1, len(currentIds) + 1):
            notFound = next((x for x in currentIds if x == possibleId), True)
            if notFound:
                id = possibleId
                break
        self.data['id'] = id

        # save to new file
        dataFile = open(self.dataFilePath, 'x')
        json.dump(self.data, dataFile)
        dataFile.close()

        itemData = {
            "fileName": self.fileName,
            "filePath": self.dataFilePath,
            "id": id,
        }

        innerCommunication.addRobotSignal.emit(itemData)
        self.goBack()

    def updateRobotData(self):
        dataFile = open(self.dataFilePath, 'r')
        data = json.load(dataFile)
        dataFile.close()
        dataFile = open(self.dataFilePath, 'w')
        data['robotName'] = self.robotNameInput.text()
        data['controlKeys']['forward'] = self.forwardKeyInput.text()
        data['controlKeys']['right'] = self.rightKeyInput.text()
        data['controlKeys']['backward'] = self.backwardKeyInput.text()
        data['controlKeys']['left'] = self.leftKeyInput.text()
        data['controlKeys']['stable'] = self.stableKeyInput.text()


        # FORWARD
        data['dynamic']['forward']['leftEngine'] = self.dynamicTable.item(0, 0).text()
        print(self.dynamicTable.item(0, 1).text())
        print(self.dynamicTable.item(1, 0).text())
        data['dynamic']['forward']['rightEngine'] = self.dynamicTable.item(0, 1).text()
        # data['dynamic']['forward']['inertia'] = self.dynamicTable.item(0, 2).text()

        data['dynamic']['forwardLeft']['leftEngine'] = self.dynamicTable.item(1, 0).text()
        data['dynamic']['forwardLeft']['rightEngine'] = self.dynamicTable.item(1, 1).text()
        # data['dynamic']['forwardLeft']['inertia'] = self.dynamicTable.item(1, 2).text()

        data['dynamic']['forwardRight']['leftEngine'] = self.dynamicTable.item(2, 0).text()
        data['dynamic']['forwardRight']['rightEngine'] = self.dynamicTable.item(2, 1).text()
        # data['dynamic']['forwardRight']['inertia'] = self.dynamicTable.item(2, 2).text()

        # RIGHT
        data['dynamic']['backward']['leftEngine'] = self.dynamicTable.item(4, 0).text()
        data['dynamic']['backward']['rightEngine'] = self.dynamicTable.item(4, 1).text()
        # data['dynamic']['backward']['inertia'] = self.dynamicTable.item(4, 2).text()

        # BACKWARD
        data['dynamic']['backwardLeft']['leftEngine'] = self.dynamicTable.item(4, 0).text()
        data['dynamic']['backwardLeft']['rightEngine'] = self.dynamicTable.item(4, 1).text()
        # data['dynamic']['backwardLeft']['inertia'] = self.dynamicTable.item(4, 2).text()

        data['dynamic']['backwardRight']['leftEngine'] = self.dynamicTable.itemAt(5, 0).text()
        data['dynamic']['backwardRight']['rightEngine'] = self.dynamicTable.itemAt(5, 1).text()
        # data['dynamic']['backwardRight']['inertia'] = self.dynamicTable.item(5, 2).text()

        data['dynamic']['backwardRight']['leftEngine'] = self.dynamicTable.itemAt(6, 0).text()
        data['dynamic']['backwardRight']['rightEngine'] = self.dynamicTable.itemAt(6, 1).text()
        # data['dynamic']['backwardRight']['inertia'] = self.dynamicTable.item(6, 2).text()

        # LEFT
        data['dynamic']['backwardRight']['leftEngine'] = self.dynamicTable.itemAt(7, 0).text()
        data['dynamic']['backwardRight']['rightEngine'] = self.dynamicTable.itemAt(7, 1).text()
        # data['dynamic']['backwardRight']['inertia'] = self.dynamicTable.item(7, 2).text()

        json.dump(data, dataFile)
        dataFile.close()

        id = data['id']

        itemData = {
            "fileName": None,
            "filePath": self.dataFilePath,
            "id": id,
        }

        print('updateRobotData')
        innerCommunication.updateRobotSignal.emit(itemData)
        self.goBack()

    def resizeEvent(self, event):
        pass
        # print("resize")

    def loadData(self, dataFilePath):
        dataFile = open(dataFilePath)
        data = json.load(dataFile)
        dataFile.close()
        return data

    def loadJson(self):
        self.robotNameInput.setText(self.data['robotName'])

        # CONTROL KEYS
        self.controlKeys = self.data['controlKeys']
        self.forwardKeyInput.setText(self.controlKeys[ControlKeyEnum.FORWARD])
        self.rightKeyInput.setText(self.controlKeys[ControlKeyEnum.RIGHT])
        self.backwardKeyInput.setText(self.controlKeys[ControlKeyEnum.BACKWARD])
        self.leftKeyInput.setText(self.controlKeys[ControlKeyEnum.LEFT])
        self.stableKeyInput.setText(self.controlKeys[ControlKeyEnum.STABLE])

        # DYNAMIC
        dynamic = self.data['dynamic']
        forwardDynamic = dynamic['forward']
        forwardLeftDynamic = dynamic['forwardLeft']
        forwardRightDynamic = dynamic['forwardRight']

        backwardDynamic = dynamic['backward']
        backwardLeftDynamic = dynamic['backward']
        backwardRightDynamic = dynamic['backward']

        # FORWARD
        self.dynamicTable.setItem(0, 0, QTableWidgetItem(str(forwardDynamic['leftEngine'])))
        self.dynamicTable.setItem(0, 1, QTableWidgetItem(str(forwardDynamic['rightEngine'])))
        self.dynamicTable.setItem(0, 2, QTableWidgetItem(str(forwardDynamic['inertia'])))

        self.dynamicTable.setItem(1, 0, QTableWidgetItem(str(forwardLeftDynamic['leftEngine'])))
        self.dynamicTable.setItem(1, 1, QTableWidgetItem(str(forwardLeftDynamic['rightEngine'])))
        self.dynamicTable.setItem(1, 2, QTableWidgetItem(str(forwardLeftDynamic['inertia'])))

        self.dynamicTable.setItem(2, 0, QTableWidgetItem(str(forwardRightDynamic['leftEngine'])))
        self.dynamicTable.setItem(2, 1, QTableWidgetItem(str(forwardRightDynamic['rightEngine'])))
        self.dynamicTable.setItem(2, 2, QTableWidgetItem(str(forwardRightDynamic['inertia'])))

        # RIGHT
        self.dynamicTable.setItem(3, 0, QTableWidgetItem(str(backwardDynamic['leftEngine'])))
        self.dynamicTable.setItem(3, 1, QTableWidgetItem(str(backwardDynamic['rightEngine'])))
        self.dynamicTable.setItem(3, 2, QTableWidgetItem(str(backwardDynamic['inertia'])))

        # BACKWARD
        self.dynamicTable.setItem(4, 0, QTableWidgetItem(str(backwardLeftDynamic['leftEngine'])))
        self.dynamicTable.setItem(4, 1, QTableWidgetItem(str(backwardLeftDynamic['rightEngine'])))
        self.dynamicTable.setItem(4, 2, QTableWidgetItem(str(backwardLeftDynamic['inertia'])))

        self.dynamicTable.setItem(5, 0, QTableWidgetItem(str(backwardRightDynamic['leftEngine'])))
        self.dynamicTable.setItem(5, 1, QTableWidgetItem(str(backwardRightDynamic['rightEngine'])))
        self.dynamicTable.setItem(5, 2, QTableWidgetItem(str(backwardRightDynamic['inertia'])))

        self.dynamicTable.setItem(6, 0, QTableWidgetItem(str(backwardRightDynamic['leftEngine'])))
        self.dynamicTable.setItem(6, 1, QTableWidgetItem(str(backwardRightDynamic['rightEngine'])))
        self.dynamicTable.setItem(6, 2, QTableWidgetItem(str(backwardRightDynamic['inertia'])))

        # LEFT
        self.dynamicTable.setItem(7, 0, QTableWidgetItem(str(backwardRightDynamic['leftEngine'])))
        self.dynamicTable.setItem(7, 1, QTableWidgetItem(str(backwardRightDynamic['rightEngine'])))
        self.dynamicTable.setItem(7, 2, QTableWidgetItem(str(backwardRightDynamic['inertia'])))

        # JOYSTICK
        joystick = self.data['joystick']

        joystickForward = joystick['forward']
        joystickRight = joystick['right']
        joystickBackward = joystick['backward']
        joystickLeft = joystick['left']

        self.joystickForward.setItem(0, 0, QTableWidgetItem(str(joystickForward['leftEngine'])))
        self.joystickForward.setItem(0, 1, QTableWidgetItem(str(joystickForward['rightEngine'])))

        self.joystickRight.setItem(0, 0, QTableWidgetItem(str(joystickRight['leftEngine'])))
        self.joystickRight.setItem(0, 1, QTableWidgetItem(str(joystickRight['rightEngine'])))

        self.joystickBackward.setItem(0, 0, QTableWidgetItem(str(joystickBackward['leftEngine'])))
        self.joystickBackward.setItem(0, 1, QTableWidgetItem(str(joystickBackward['rightEngine'])))

        self.joystickLeft.setItem(0, 0, QTableWidgetItem(str(joystickLeft['leftEngine'])))
        self.joystickLeft.setItem(0, 1, QTableWidgetItem(str(joystickLeft['rightEngine'])))
