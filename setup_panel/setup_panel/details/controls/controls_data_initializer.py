from python_qt_binding.QtWidgets import QTableWidgetItem
from shared.enums import ControlKeyEnum
from shared.enums import MotorControlPositionEnum, motorControlPositionToDataKeyMap


class ControlsDataInitializer:
    def __init__(self, widget, controlKeys, data):
        self.widget = widget
        self.controlKeys = controlKeys
        self.data = data
        self.setData()

    def setData(self):
        self.setRobotName()
        self.setNamespace()
        self.setControlKeys()
        self.setMotorsDynamic()
        self.setJoystick()

    def setRobotName(self):
        self.widget.robotNameInput.setText(self.data.get('robotName', ''))

    def setNamespace(self):
        self.widget.namespaceLineEditUI.setText(self.data.get('namespace', ''))

    def setControlKeys(self):
        self.widget.forwardKeyInput.setText(self.controlKeys.get(ControlKeyEnum.FORWARD))
        self.widget.rightKeyInput.setText(self.controlKeys.get(ControlKeyEnum.RIGHT))
        self.widget.backwardKeyInput.setText(self.controlKeys.get(ControlKeyEnum.BACKWARD))
        self.widget.leftKeyInput.setText(self.controlKeys.get(ControlKeyEnum.LEFT))
        self.widget.stableKeyInput.setText(self.controlKeys.get(ControlKeyEnum.STABLE))

    def setJoystick(self):
        joystick = self.data.get('joystick', {})
        joystickMotorCommand = joystick.get('motorCommand', {})
        joystickTwist = joystick.get('twist', {})

        joystickForward = joystickMotorCommand.get('forward', {})
        joystickRight = joystickMotorCommand.get('right', {})
        joystickBackward = joystickMotorCommand.get('backward', {})
        joystickLeft = joystickMotorCommand.get('left', {})

        self.widget.joystickForward.setItem(0, 0, QTableWidgetItem(str(joystickForward.get('leftEngine', 0))))
        self.widget.joystickForward.setItem(0, 1, QTableWidgetItem(str(joystickForward.get('rightEngine', 0))))

        self.widget.joystickRight.setItem(0, 0, QTableWidgetItem(str(joystickRight.get('leftEngine', 0))))
        self.widget.joystickRight.setItem(0, 1, QTableWidgetItem(str(joystickRight.get('rightEngine', 0))))

        self.widget.joystickBackward.setItem(0, 0, QTableWidgetItem(str(joystickBackward.get('leftEngine', 0))))
        self.widget.joystickBackward.setItem(0, 1, QTableWidgetItem(str(joystickBackward.get('rightEngine', 0))))

        self.widget.joystickLeft.setItem(0, 0, QTableWidgetItem(str(joystickLeft.get('leftEngine', 0))))
        self.widget.joystickLeft.setItem(0, 1, QTableWidgetItem(str(joystickLeft.get('rightEngine', 0))))

        self.widget.joystickTwistTableUI.setItem(0, 0, QTableWidgetItem(str(joystickTwist.get('linear', 0))))
        self.widget.joystickTwistTableUI.setItem(1, 0, QTableWidgetItem(str(joystickTwist.get('angular', 0))))

    def setMotorsDynamic(self):
        dynamic = self.data.get('dynamic', {})
        dynamicTwist = self.data.get('dynamicTwist', {})

        for MotorControlPosition in MotorControlPositionEnum:
            self.setControlsForKeyCombination(MotorControlPosition, dynamic)
            self.setTwistControlsForKeyCombination(MotorControlPosition, dynamicTwist)

    def setControlsForKeyCombination(self, motorControlPositionEnum, dynamic):
        dataKey = motorControlPositionToDataKeyMap[motorControlPositionEnum]
        data = dynamic.get(dataKey, {})

        setDynamicTableItem = self.widget.dynamicTable.setItem
        setDynamicTableItem(motorControlPositionEnum, 0, self.createTableItem(data, 'leftEngine'))
        setDynamicTableItem(motorControlPositionEnum, 1, self.createTableItem(data, 'rightEngine'))

    def setTwistControlsForKeyCombination(self, motorControlPositionEnum, dynamicTwist):
        dataKey = motorControlPositionToDataKeyMap[motorControlPositionEnum]
        data = dynamicTwist.get(dataKey, {})

        setDynamicTableItem = self.widget.dynamicTwistTableUI.setItem
        setDynamicTableItem(motorControlPositionEnum, 0, self.createTableItem(data, 'linear'))
        setDynamicTableItem(motorControlPositionEnum, 1, self.createTableItem(data, 'angular'))

    def createTableItem(self, data, key, default=None):
        return QTableWidgetItem(str(data.get(key, default)))
