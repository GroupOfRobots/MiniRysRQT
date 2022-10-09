from shared.enums import ControlKeyEnum
from shared.enums import MotorControlPositionInTableEnum

from python_qt_binding.QtWidgets import QTableWidgetItem


class ControlsDataInitializer:
    def __init__(self, widget, controlKeys, data):
        self.widget = widget
        self.controlKeys = controlKeys
        self.data = data
        self.setData()

    def setData(self):
        self.setRobotName()
        self.setControlKeys()
        self.setMotorsDynamic()
        self.setJoystick()

    def setRobotName(self):
        self.widget.robotNameInput.setText(self.data['robotName'])

    def setControlKeys(self):
        self.widget.forwardKeyInput.setText(self.controlKeys.get(ControlKeyEnum.FORWARD))
        self.widget.rightKeyInput.setText(self.controlKeys.get(ControlKeyEnum.RIGHT))
        self.widget.backwardKeyInput.setText(self.controlKeys.get(ControlKeyEnum.BACKWARD))
        self.widget.leftKeyInput.setText(self.controlKeys.get(ControlKeyEnum.LEFT))
        self.widget.stableKeyInput.setText(self.controlKeys.get(ControlKeyEnum.STABLE))

    def setJoystick(self):
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

    def setMotorsDynamic(self):
        dynamic = self.data.get('dynamic', {})

        self.setForwardMotorsCombinations(dynamic)
        self.setRightMotorsCombinations(dynamic)
        self.setBackwardMotorsCombinations(dynamic)
        self.setLeftMotorsCombinations(dynamic)

    def setForwardMotorsCombinations(self, dynamic):
        forwardDynamic = dynamic.get('forward', {})
        forwardLeftDynamic = dynamic.get('forwardLeft', {})
        forwardRightDynamic = dynamic.get('forwardRight', {})

        self.setControlsForKeyCombination(MotorControlPositionInTableEnum.FORWARD, forwardDynamic)
        self.setControlsForKeyCombination(MotorControlPositionInTableEnum.FORWARD_LEFT, forwardLeftDynamic)
        self.setControlsForKeyCombination(MotorControlPositionInTableEnum.FORWARD_RIGHT, forwardRightDynamic)

    def setRightMotorsCombinations(self, dynamic):
        rightDynamic = dynamic.get('right', {})

        self.setControlsForKeyCombination(MotorControlPositionInTableEnum.RIGHT, rightDynamic)

    def setBackwardMotorsCombinations(self, dynamic):
        backwardDynamic = dynamic.get('backward', {})
        backwardLeftDynamic = dynamic.get('backwardLeft', {})
        backwardRightDynamic = dynamic.get('backwardRight', {})

        self.setControlsForKeyCombination(MotorControlPositionInTableEnum.BACKWARD, backwardDynamic)
        self.setControlsForKeyCombination(MotorControlPositionInTableEnum.BACKWARD_LEFT, backwardLeftDynamic)
        self.setControlsForKeyCombination(MotorControlPositionInTableEnum.BACKWARD_RIGHT, backwardRightDynamic)

    def setLeftMotorsCombinations(self, dynamic):
        leftDynamic = dynamic.get('left', {})

        self.setControlsForKeyCombination(MotorControlPositionInTableEnum.LEFT, leftDynamic)

    def setControlsForKeyCombination(self, motorControlPositionInTableEnum, data):
        setDynamicTableItem = self.widget.dynamicTable.setItem
        setDynamicTableItem(motorControlPositionInTableEnum, 0, self.createTableItem(data, 'leftEngine'))
        setDynamicTableItem(motorControlPositionInTableEnum, 1, self.createTableItem(data, 'rightEngine'))
        setDynamicTableItem(motorControlPositionInTableEnum, 2, self.createTableItem(data, 'inertia'))

    def createTableItem(self, data, key, default=None):
        return QTableWidgetItem(str(data.get(key, default)))
