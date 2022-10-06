from python_qt_binding.QtWidgets import QWidget, QTableWidgetItem
from shared.enums import ControlKeyEnum
from enum import Enum


class Controls(QWidget):
    def __init__(self, widget):
        super(Controls, self).__init__()
        self.widget = widget
        self.data = self.widget.data

        self.keyInputDictionary = {
            ControlKeyEnum.FORWARD: self.widget.forwardKeyInput,
            ControlKeyEnum.RIGHT: self.widget.rightKeyInput,
            ControlKeyEnum.BACKWARD: self.widget.backwardKeyInput,
            ControlKeyEnum.LEFT: self.widget.leftKeyInput,
            ControlKeyEnum.STABLE: self.widget.stableKeyInput,
        }

        self.controlKeys = self.data.get('controlKeys', {})

        self.setData()

        self.addControlKeysValidators()

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
                self.keyInputDictionary[buttonKeys[0]].setStyleSheet('background: lightGray;')

        if disableSaveButton:
            self.widget.saveButton.setEnabled(False)
        else:
            self.widget.saveButton.setEnabled(True)

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

    def setData(self):
        self.widget.robotNameInput.setText(self.data['robotName'])

        # CONTROL KEYS
        self.widget.forwardKeyInput.setText(self.controlKeys.get(ControlKeyEnum.FORWARD))
        self.widget.rightKeyInput.setText(self.controlKeys.get(ControlKeyEnum.RIGHT))
        self.widget.backwardKeyInput.setText(self.controlKeys.get(ControlKeyEnum.BACKWARD))
        self.widget.leftKeyInput.setText(self.controlKeys.get(ControlKeyEnum.LEFT))
        self.widget.stableKeyInput.setText(self.controlKeys.get(ControlKeyEnum.STABLE))

        # DYNAMIC
        dynamic = self.data.get('dynamic', {})
        forwardDynamic = dynamic.get('forward', {})
        forwardLeftDynamic = dynamic.get('forwardLeft', {})
        forwardRightDynamic = dynamic.get('forwardRight', {})

        rightDynamic = dynamic.get('right', {})

        backwardDynamic = dynamic.get('backward', {})
        backwardLeftDynamic = dynamic.get('backwardLeft', {})
        backwardRightDynamic = dynamic.get('backwardRight', {})

        leftDynamic = dynamic.get('left', {})
        print(dynamic)

        # FORWARD
        self.widget.dynamicTable.setItem(MotorControlPositionInTableEnum.FORWARD, 0,
                                         QTableWidgetItem(str(forwardDynamic.get('leftEngine'))))
        self.widget.dynamicTable.setItem(MotorControlPositionInTableEnum.FORWARD, 1,
                                         QTableWidgetItem(str(forwardDynamic.get('rightEngine'))))
        self.widget.dynamicTable.setItem(MotorControlPositionInTableEnum.FORWARD, 2,
                                         QTableWidgetItem(str(forwardDynamic.get('inertia'))))

        self.widget.dynamicTable.setItem(MotorControlPositionInTableEnum.FORWARD_LEFT, 0,
                                         QTableWidgetItem(str(forwardLeftDynamic.get('leftEngine'))))
        self.widget.dynamicTable.setItem(MotorControlPositionInTableEnum.FORWARD_LEFT, 1,
                                         QTableWidgetItem(str(forwardLeftDynamic.get('rightEngine'))))
        self.widget.dynamicTable.setItem(MotorControlPositionInTableEnum.FORWARD_LEFT, 2,
                                         QTableWidgetItem(str(forwardLeftDynamic.get('inertia'))))

        self.widget.dynamicTable.setItem(MotorControlPositionInTableEnum.FORWARD_RIGHT, 0,
                                         QTableWidgetItem(str(forwardRightDynamic.get('leftEngine'))))
        self.widget.dynamicTable.setItem(MotorControlPositionInTableEnum.FORWARD_RIGHT, 1,
                                         QTableWidgetItem(str(forwardRightDynamic.get('rightEngine'))))
        self.widget.dynamicTable.setItem(MotorControlPositionInTableEnum.FORWARD_RIGHT, 2,
                                         QTableWidgetItem(str(forwardRightDynamic.get('inertia'))))

        # RIGHT
        self.widget.dynamicTable.setItem(MotorControlPositionInTableEnum.RIGHT, 0,
                                         QTableWidgetItem(str(rightDynamic.get('leftEngine'))))
        self.widget.dynamicTable.setItem(MotorControlPositionInTableEnum.RIGHT, 1,
                                         QTableWidgetItem(str(rightDynamic.get('rightEngine'))))
        self.widget.dynamicTable.setItem(MotorControlPositionInTableEnum.RIGHT, 2,
                                         QTableWidgetItem(str(rightDynamic.get('inertia'))))

        # BACKWARD
        self.widget.dynamicTable.setItem(MotorControlPositionInTableEnum.BACKWARD, 0,
                                         QTableWidgetItem(str(backwardDynamic.get('leftEngine'))))
        self.widget.dynamicTable.setItem(MotorControlPositionInTableEnum.BACKWARD, 1,
                                         QTableWidgetItem(str(backwardDynamic.get('rightEngine'))))
        self.widget.dynamicTable.setItem(MotorControlPositionInTableEnum.BACKWARD, 2,
                                         QTableWidgetItem(str(backwardDynamic.get('inertia'))))

        self.widget.dynamicTable.setItem(MotorControlPositionInTableEnum.BACKWARD_LEFT, 0,
                                         QTableWidgetItem(str(backwardLeftDynamic.get('leftEngine'))))
        self.widget.dynamicTable.setItem(MotorControlPositionInTableEnum.BACKWARD_LEFT, 1,
                                         QTableWidgetItem(str(backwardLeftDynamic.get('rightEngine'))))
        self.widget.dynamicTable.setItem(MotorControlPositionInTableEnum.BACKWARD_LEFT, 2,
                                         QTableWidgetItem(str(backwardLeftDynamic.get('inertia'))))

        self.widget.dynamicTable.setItem(MotorControlPositionInTableEnum.BACKWARD_RIGHT, 0,
                                         QTableWidgetItem(str(backwardRightDynamic.get('leftEngine'))))
        self.widget.dynamicTable.setItem(MotorControlPositionInTableEnum.BACKWARD_RIGHT, 1,
                                         QTableWidgetItem(str(backwardRightDynamic.get('rightEngine'))))
        self.widget.dynamicTable.setItem(MotorControlPositionInTableEnum.BACKWARD_RIGHT, 2,
                                         QTableWidgetItem(str(backwardRightDynamic.get('inertia'))))

        # LEFT
        self.widget.dynamicTable.setItem(MotorControlPositionInTableEnum.LEFT, 0,
                                         QTableWidgetItem(str(leftDynamic.get('leftEngine'))))
        self.widget.dynamicTable.setItem(MotorControlPositionInTableEnum.LEFT, 1,
                                         QTableWidgetItem(str(leftDynamic.get('rightEngine'))))
        self.widget.dynamicTable.setItem(MotorControlPositionInTableEnum.LEFT, 2,
                                         QTableWidgetItem(str(leftDynamic.get('inertia'))))

        # JOYSTICK
        joystick = self.data['joystick']

        joystickForward = joystick['forward']
        joystickRight = joystick['right']
        joystickBackward = joystick['backward']
        joystickLeft = joystick['left']

        self.widget.joystickForward.setItem(0, 0, QTableWidgetItem(str(joystickForward['leftEngine'])))
        self.widget.joystickForward.setItem(0, 1, QTableWidgetItem(str(joystickForward['rightEngine'])))

        self.widget.joystickRight.setItem(0, 0, QTableWidgetItem(str(joystickRight['leftEngine'])))
        self.widget.joystickRight.setItem(0, 1, QTableWidgetItem(str(joystickRight['rightEngine'])))

        self.widget.joystickBackward.setItem(0, 0, QTableWidgetItem(str(joystickBackward['leftEngine'])))
        self.widget.joystickBackward.setItem(0, 1, QTableWidgetItem(str(joystickBackward['rightEngine'])))

        self.widget.joystickLeft.setItem(0, 0, QTableWidgetItem(str(joystickLeft['leftEngine'])))
        self.widget.joystickLeft.setItem(0, 1, QTableWidgetItem(str(joystickLeft['rightEngine'])))

    def saveControls(self):
        self.widget.data['robotName'] = self.widget.robotNameInput.text()
        self.widget.data['controlKeys']['forward'] = self.widget.forwardKeyInput.text()
        self.widget.data['controlKeys']['backward'] = self.widget.backwardKeyInput.text()
        self.widget.data['controlKeys']['right'] = self.widget.rightKeyInput.text()
        self.widget.data['controlKeys']['left'] = self.widget.leftKeyInput.text()
        self.widget.data['controlKeys']['stable'] = self.widget.stableKeyInput.text()

        # FORWARD
        self.widget.data['dynamic']['forward']['leftEngine'] = self.widget.dynamicTable.item(
            MotorControlPositionInTableEnum.FORWARD, 0).text()
        self.widget.data['dynamic']['forward']['rightEngine'] = self.widget.dynamicTable.item(
            MotorControlPositionInTableEnum.FORWARD, 1).text()

        self.widget.data['dynamic']['forwardLeft']['leftEngine'] = self.widget.dynamicTable.item(
            MotorControlPositionInTableEnum.FORWARD_LEFT, 0).text()
        self.widget.data['dynamic']['forwardLeft']['rightEngine'] = self.widget.dynamicTable.item(
            MotorControlPositionInTableEnum.FORWARD_LEFT, 1).text()

        self.widget.data['dynamic']['forwardRight']['leftEngine'] = self.widget.dynamicTable.item(
            MotorControlPositionInTableEnum.FORWARD_RIGHT, 0).text()
        self.widget.data['dynamic']['forwardRight']['rightEngine'] = self.widget.dynamicTable.item(
            MotorControlPositionInTableEnum.FORWARD_RIGHT, 1).text()

        # RIGHT
        self.widget.data['dynamic']['right']['leftEngine'] = self.widget.dynamicTable.item(
            MotorControlPositionInTableEnum.RIGHT, 0).text()
        self.widget.data['dynamic']['right']['rightEngine'] = self.widget.dynamicTable.item(
            MotorControlPositionInTableEnum.RIGHT, 1).text()

        # BACKWARD
        self.widget.data['dynamic']['backward']['leftEngine'] = self.widget.dynamicTable.item(
            MotorControlPositionInTableEnum.BACKWARD, 0).text()
        self.widget.data['dynamic']['backward']['rightEngine'] = self.widget.dynamicTable.item(
            MotorControlPositionInTableEnum.BACKWARD, 1).text()

        self.widget.data['dynamic']['backwardLeft']['leftEngine'] = self.widget.dynamicTable.item(
            MotorControlPositionInTableEnum.BACKWARD_LEFT, 0).text()
        self.widget.data['dynamic']['backwardLeft']['rightEngine'] = self.widget.dynamicTable.item(
            MotorControlPositionInTableEnum.BACKWARD_LEFT, 1).text()

        self.widget.data['dynamic']['backwardRight']['leftEngine'] = self.widget.dynamicTable.item(
            MotorControlPositionInTableEnum.BACKWARD_RIGHT, 0).text()
        self.widget.data['dynamic']['backwardRight']['rightEngine'] = self.widget.dynamicTable.item(
            MotorControlPositionInTableEnum.BACKWARD_RIGHT, 1).text()

        # LEFT
        self.widget.data['dynamic']['left']['leftEngine'] = self.widget.dynamicTable.item(
            MotorControlPositionInTableEnum.LEFT, 0).text()
        self.widget.data['dynamic']['left']['rightEngine'] = self.widget.dynamicTable.item(
            MotorControlPositionInTableEnum.LEFT, 1).text()

        # print('lf.widget.dynamicTable.items()')
        # print(self.widget.dynamicTable.items())


class MotorControlPositionInTableEnum(int, Enum):
    FORWARD = 0
    FORWARD_LEFT = 1
    FORWARD_RIGHT = 2
    RIGHT = 3
    BACKWARD = 4  # position is preserved if stable key is pressed
    BACKWARD_LEFT = 5
    BACKWARD_RIGHT = 6
    LEFT = 7
