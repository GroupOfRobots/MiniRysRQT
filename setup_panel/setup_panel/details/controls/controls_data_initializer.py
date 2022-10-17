from shared.enums import ControlKeyEnum
from shared.enums import MotorControlPositionInTableEnum, motorControlPositionToDataKeyMap

from python_qt_binding.QtWidgets import QTableWidgetItem


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
        dynamicTwist = self.data.get('dynamicTwist', {})

        for motorControlPositionInTable in MotorControlPositionInTableEnum:
            self.setControlsForKeyCombination(motorControlPositionInTable, dynamic)
            self.setTwistControlsForKeyCombination(motorControlPositionInTable, dynamicTwist)

    def setControlsForKeyCombination(self, motorControlPositionInTableEnum, dynamic):
        dataKey = motorControlPositionToDataKeyMap[motorControlPositionInTableEnum]
        data = dynamic.get(dataKey, {})

        setDynamicTableItem = self.widget.dynamicTable.setItem
        setDynamicTableItem(motorControlPositionInTableEnum, 0, self.createTableItem(data, 'leftEngine'))
        setDynamicTableItem(motorControlPositionInTableEnum, 1, self.createTableItem(data, 'rightEngine'))
        setDynamicTableItem(motorControlPositionInTableEnum, 2, self.createTableItem(data, 'inertia'))

    def setTwistControlsForKeyCombination(self, motorControlPositionInTableEnum, dynamicTwist):
        dataKey = motorControlPositionToDataKeyMap[motorControlPositionInTableEnum]
        data = dynamicTwist.get(dataKey, {})

        setDynamicTableItem = self.widget.dynamicTwistTableUI.setItem
        setDynamicTableItem(motorControlPositionInTableEnum, 0, self.createTableItem(data, 'linear'))
        setDynamicTableItem(motorControlPositionInTableEnum, 1, self.createTableItem(data, 'angle'))
        # setDynamicTableItem(motorControlPositionInTableEnum, 2, self.createTableItem(data, 'inertia'))

    def createTableItem(self, data, key, default=None):
        return QTableWidgetItem(str(data.get(key, default)))
