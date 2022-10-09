from python_qt_binding.QtWidgets import QWidget, QTableWidgetItem
from shared.enums import ControlKeyEnum
from enum import Enum
from .control_key_validator import ControlKeyValidator
from .controls_data_initializer import ControlsDataInitializer


class Controls(QWidget):
    def __init__(self, widget):
        super(Controls, self).__init__()
        self.widget = widget
        self.data = self.widget.data

        self.controlKeys = self.data.get('controlKeys', {})

        self.controlKeyValidator = ControlKeyValidator(self.widget, self.controlKeys)
        self.controlsDataInitializer = ControlsDataInitializer(self.widget, self.controlKeys, self.data)

    def saveControls(self):
        data = self.widget.data
        data['robotName'] = self.widget.robotNameInput.text()
        data['controlKeys']['forward'] = self.widget.forwardKeyInput.text()
        data['controlKeys']['backward'] = self.widget.backwardKeyInput.text()
        data['controlKeys']['right'] = self.widget.rightKeyInput.text()
        data['controlKeys']['left'] = self.widget.leftKeyInput.text()
        data['controlKeys']['stable'] = self.widget.stableKeyInput.text()

        # FORWARD
        data['dynamic']['forward']['leftEngine'] = self.widget.dynamicTable.item(
            MotorControlPositionInTableEnum.FORWARD, 0).text()
        data['dynamic']['forward']['rightEngine'] = self.widget.dynamicTable.item(
            MotorControlPositionInTableEnum.FORWARD, 1).text()

        data['dynamic']['forwardLeft']['leftEngine'] = self.widget.dynamicTable.item(
            MotorControlPositionInTableEnum.FORWARD_LEFT, 0).text()
        data['dynamic']['forwardLeft']['rightEngine'] = self.widget.dynamicTable.item(
            MotorControlPositionInTableEnum.FORWARD_LEFT, 1).text()

        data['dynamic']['forwardRight']['leftEngine'] = self.widget.dynamicTable.item(
            MotorControlPositionInTableEnum.FORWARD_RIGHT, 0).text()
        data['dynamic']['forwardRight']['rightEngine'] = self.widget.dynamicTable.item(
            MotorControlPositionInTableEnum.FORWARD_RIGHT, 1).text()

        # RIGHT
        data['dynamic']['right']['leftEngine'] = self.widget.dynamicTable.item(
            MotorControlPositionInTableEnum.RIGHT, 0).text()
        data['dynamic']['right']['rightEngine'] = self.widget.dynamicTable.item(
            MotorControlPositionInTableEnum.RIGHT, 1).text()

        # BACKWARD
        data['dynamic']['backward']['leftEngine'] = self.widget.dynamicTable.item(
            MotorControlPositionInTableEnum.BACKWARD, 0).text()
        data['dynamic']['backward']['rightEngine'] = self.widget.dynamicTable.item(
            MotorControlPositionInTableEnum.BACKWARD, 1).text()

        data['dynamic']['backwardLeft']['leftEngine'] = self.widget.dynamicTable.item(
            MotorControlPositionInTableEnum.BACKWARD_LEFT, 0).text()
        data['dynamic']['backwardLeft']['rightEngine'] = self.widget.dynamicTable.item(
            MotorControlPositionInTableEnum.BACKWARD_LEFT, 1).text()

        data['dynamic']['backwardRight']['leftEngine'] = self.widget.dynamicTable.item(
            MotorControlPositionInTableEnum.BACKWARD_RIGHT, 0).text()
        data['dynamic']['backwardRight']['rightEngine'] = self.widget.dynamicTable.item(
            MotorControlPositionInTableEnum.BACKWARD_RIGHT, 1).text()

        # LEFT
        data['dynamic']['left']['leftEngine'] = self.widget.dynamicTable.item(
            MotorControlPositionInTableEnum.LEFT, 0).text()
        data['dynamic']['left']['rightEngine'] = self.widget.dynamicTable.item(
            MotorControlPositionInTableEnum.LEFT, 1).text()


class MotorControlPositionInTableEnum(int, Enum):
    FORWARD = 0
    FORWARD_LEFT = 1
    FORWARD_RIGHT = 2
    RIGHT = 3
    BACKWARD = 4  # position is preserved if stable key is pressed
    BACKWARD_LEFT = 5
    BACKWARD_RIGHT = 6
    LEFT = 7
