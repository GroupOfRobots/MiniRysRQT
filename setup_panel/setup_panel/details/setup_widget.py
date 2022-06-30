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
            self.loadData(self.dataFilePath)
        else:
            self.addMode = True
            self.dataPath = os.path.join(self.sharedPath, 'share', 'shared', 'data', 'robots')
            self.loadData(self.defaultFilePath)

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

        self.setupCommandsElements()

        self.initSshData()

        self.initPidData()

        self.pidSpeedKpUI.valueChanged.connect(self.test)

    def initSshData(self):
        self.sshData = SshData(self)

        self.sshHostInputUI.editingFinished.connect(self.sshData.hostEdited)
        self.sshPortInputUI.editingFinished.connect(self.sshData.portEdited)
        self.sshUsernameInputUI.editingFinished.connect(self.sshData.usernameEdited)
        self.sshPasswordInputUI.editingFinished.connect(self.sshData.passwordEdited)

    def initPidData(self):
        self.pidWidget = PidWidget(self)

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

    def setupCommandsElements(self):
        self.addNewCommandButtonUI.clicked.connect(self.addNewCommand)
        commands = self.data.get('commands', [])
        for command in commands:
            commandElement = CommandElementWidget(self, command)
            self.commandElementsBoxUI.addWidget(commandElement)

    def addNewCommand(self):
        commandElement = CommandElementWidget(self, None)
        self.commandElementsBoxUI.addWidget(commandElement)

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
        self.saveCommands()

        if self.addMode:
            self.addNewRobot()
            return
        self.updateRobotData()

    def saveCommands(self):
        self.data['commands'] = []
        for index in range(0, self.commandElementsBoxUI.count()):
            commandElement = self.commandElementsBoxUI.itemAt(index).widget()
            self.data['commands'].append(commandElement.returnCommand())

    def addNewRobot(self):
        print('addNewRobot')
        print(self.data)
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
        print('updateRobotData')

        # dataFile = open(self.dataFilePath, 'r')
        # data = json.load(dataFile)
        # dataFile.close()
        # dataFile = open(self.dataFilePath, 'w')
        self.data['robotName'] = self.robotNameInput.text()
        self.data['controlKeys']['forward'] = self.forwardKeyInput.text()
        self.data['controlKeys']['backward'] = self.backwardKeyInput.text()
        self.data['controlKeys']['right'] = self.rightKeyInput.text()
        self.data['controlKeys']['left'] = self.leftKeyInput.text()
        self.data['controlKeys']['stable'] = self.stableKeyInput.text()

        # FORWARD
        self.data['dynamic']['forward']['leftEngine'] = self.dynamicTable.item(0, 0).text()
        print(self.dynamicTable.item(0, 1).text())
        print(self.dynamicTable.item(1, 0).text())
        self.data['dynamic']['forward']['rightEngine'] = self.dynamicTable.item(0, 1).text()
        # data['dynamic']['forward']['inertia'] = self.dynamicTable.item(0, 2).text()

        self.data['dynamic']['forwardLeft']['leftEngine'] = self.dynamicTable.item(1, 0).text()
        self.data['dynamic']['forwardLeft']['rightEngine'] = self.dynamicTable.item(1, 1).text()
        # data['dynamic']['forwardLeft']['inertia'] = self.dynamicTable.item(1, 2).text()

        self.data['dynamic']['forwardRight']['leftEngine'] = self.dynamicTable.item(2, 0).text()
        self.data['dynamic']['forwardRight']['rightEngine'] = self.dynamicTable.item(2, 1).text()
        # data['dynamic']['forwardRight']['inertia'] = self.dynamicTable.item(2, 2).text()

        # RIGHT
        self.data['dynamic']['backward']['leftEngine'] = self.dynamicTable.item(4, 0).text()
        self.data['dynamic']['backward']['rightEngine'] = self.dynamicTable.item(4, 1).text()
        # data['dynamic']['backward']['inertia'] = self.dynamicTable.item(4, 2).text()

        # BACKWARD
        self.data['dynamic']['backwardLeft']['leftEngine'] = self.dynamicTable.item(4, 0).text()
        self.data['dynamic']['backwardLeft']['rightEngine'] = self.dynamicTable.item(4, 1).text()
        # data['dynamic']['backwardLeft']['inertia'] = self.dynamicTable.item(4, 2).text()

        self.data['dynamic']['backwardRight']['leftEngine'] = self.dynamicTable.itemAt(5, 0).text()
        self.data['dynamic']['backwardRight']['rightEngine'] = self.dynamicTable.itemAt(5, 1).text()
        # data['dynamic']['backwardRight']['inertia'] = self.dynamicTable.item(5, 2).text()

        self.data['dynamic']['backwardRight']['leftEngine'] = self.dynamicTable.itemAt(6, 0).text()
        self.data['dynamic']['backwardRight']['rightEngine'] = self.dynamicTable.itemAt(6, 1).text()
        # data['dynamic']['backwardRight']['inertia'] = self.dynamicTable.item(6, 2).text()

        # LEFT
        self.data['dynamic']['backwardRight']['leftEngine'] = self.dynamicTable.itemAt(7, 0).text()
        self.data['dynamic']['backwardRight']['rightEngine'] = self.dynamicTable.itemAt(7, 1).text()
        # data['dynamic']['backwardRight']['inertia'] = self.dynamicTable.item(7, 2).text()

        # print('self.data')
        # print(self.data)

        # json.dump(self.data, dataFile)
        # dataFile.close()

        id = self.data.get('id', None)

        itemData = {
            "fileName": None,
            "filePath": self.dataFilePath,
            "id": id,
        }

        dataFile = open(self.dataFilePath, 'w')
        json.dump(self.data, dataFile)
        dataFile.close()

        print('updateRobotData')

        innerCommunication.updateRobotSignal.emit(itemData)
        self.goBack()

    def resizeEvent(self, event):
        pass
        # print("resize")

    def loadData(self, dataFilePath):
        dataFile = open(dataFilePath)
        self.data = json.load(dataFile)
        dataFile.close()

    def loadJson(self):
        self.robotNameInput.setText(self.data['robotName'])

        # CONTROL KEYS
        self.controlKeys = self.data.get('controlKeys', {})
        self.forwardKeyInput.setText(self.controlKeys.get(ControlKeyEnum.FORWARD))
        self.rightKeyInput.setText(self.controlKeys.get(ControlKeyEnum.RIGHT))
        self.backwardKeyInput.setText(self.controlKeys.get(ControlKeyEnum.BACKWARD))
        self.leftKeyInput.setText(self.controlKeys.get(ControlKeyEnum.LEFT))
        self.stableKeyInput.setText(self.controlKeys.get(ControlKeyEnum.STABLE))

        # DYNAMIC
        dynamic = self.data.get('dynamic', {})
        forwardDynamic = dynamic.get('forward', {})
        forwardLeftDynamic = dynamic.get('forwardLeft', {})
        forwardRightDynamic = dynamic.get('forwardRight', {})

        backwardDynamic = dynamic.get('backward', {})
        backwardLeftDynamic = dynamic.get('backwardLeft', {})
        backwardRightDynamic = dynamic.get('backwardRight', {})

        # FORWARD
        self.dynamicTable.setItem(0, 0, QTableWidgetItem(str(forwardDynamic.get('leftEngine'))))
        self.dynamicTable.setItem(0, 1, QTableWidgetItem(str(forwardDynamic.get('rightEngine'))))
        self.dynamicTable.setItem(0, 2, QTableWidgetItem(str(forwardDynamic.get('inertia'))))

        self.dynamicTable.setItem(1, 0, QTableWidgetItem(str(forwardLeftDynamic.get('leftEngine'))))
        self.dynamicTable.setItem(1, 1, QTableWidgetItem(str(forwardLeftDynamic.get('rightEngine'))))
        self.dynamicTable.setItem(1, 2, QTableWidgetItem(str(forwardLeftDynamic.get('inertia'))))

        self.dynamicTable.setItem(2, 0, QTableWidgetItem(str(forwardRightDynamic.get('leftEngine'))))
        self.dynamicTable.setItem(2, 1, QTableWidgetItem(str(forwardRightDynamic.get('rightEngine'))))
        self.dynamicTable.setItem(2, 2, QTableWidgetItem(str(forwardRightDynamic.get('inertia'))))

        # RIGHT
        self.dynamicTable.setItem(3, 0, QTableWidgetItem(str(backwardDynamic.get('leftEngine'))))
        self.dynamicTable.setItem(3, 1, QTableWidgetItem(str(backwardDynamic.get('rightEngine'))))
        self.dynamicTable.setItem(3, 2, QTableWidgetItem(str(backwardDynamic.get('inertia'))))

        # BACKWARD
        self.dynamicTable.setItem(4, 0, QTableWidgetItem(str(backwardLeftDynamic.get('leftEngine'))))
        self.dynamicTable.setItem(4, 1, QTableWidgetItem(str(backwardLeftDynamic.get('rightEngine'))))
        self.dynamicTable.setItem(4, 2, QTableWidgetItem(str(backwardLeftDynamic.get('inertia'))))

        self.dynamicTable.setItem(5, 0, QTableWidgetItem(str(backwardRightDynamic.get('leftEngine'))))
        self.dynamicTable.setItem(5, 1, QTableWidgetItem(str(backwardRightDynamic.get('rightEngine'))))
        self.dynamicTable.setItem(5, 2, QTableWidgetItem(str(backwardRightDynamic.get('inertia'))))

        self.dynamicTable.setItem(6, 0, QTableWidgetItem(str(backwardRightDynamic.get('leftEngine'))))
        self.dynamicTable.setItem(6, 1, QTableWidgetItem(str(backwardRightDynamic.get('rightEngine'))))
        self.dynamicTable.setItem(6, 2, QTableWidgetItem(str(backwardRightDynamic.get('inertia'))))

        # LEFT
        self.dynamicTable.setItem(7, 0, QTableWidgetItem(str(backwardRightDynamic.get('leftEngine'))))
        self.dynamicTable.setItem(7, 1, QTableWidgetItem(str(backwardRightDynamic.get('rightEngine'))))
        self.dynamicTable.setItem(7, 2, QTableWidgetItem(str(backwardRightDynamic.get('inertia'))))

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
