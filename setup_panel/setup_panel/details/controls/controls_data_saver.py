from shared.enums import ControlKeyEnum
from shared.enums import MotorControlPositionEnum, motorControlPositionToDataKeyMap

from python_qt_binding.QtWidgets import QTableWidgetItem


class ControlsDataSaver:
    def __init__(self, widget, controlKeys, data):
        self.widget = widget
        self.controlKeys = controlKeys
        self.data = data

    def save(self):
        self.saveControls()
        self.saveMotorsDynamic()

    def saveControls(self):
        data = self.widget.data
        data['robotName'] = self.widget.robotNameInput.text()
        data['namespace'] = self.widget.namespaceLineEditUI.text()
        data['controlKeys']['forward'] = self.widget.forwardKeyInput.text()
        data['controlKeys']['backward'] = self.widget.backwardKeyInput.text()
        data['controlKeys']['right'] = self.widget.rightKeyInput.text()
        data['controlKeys']['left'] = self.widget.leftKeyInput.text()
        data['controlKeys']['stable'] = self.widget.stableKeyInput.text()

    def saveMotorsDynamic(self):
        dynamic = {}
        dynamicTwist = {}

        for motorControlPosition in MotorControlPositionEnum:
            self.saveControlsForKeyCombination(motorControlPosition, dynamic)
            self.saveTwistControlsForKeyCombination(motorControlPosition, dynamicTwist)

        self.data['dynamic']= dynamic
        self.data['dynamicTwist'] = dynamicTwist

    def saveControlsForKeyCombination(self, motorControlPosition, dynamic):
        dataKey = motorControlPositionToDataKeyMap[motorControlPosition]

        dynamic[dataKey] = {}
        dynamic[dataKey]['leftEngine'] = self.saveDynamicTableItem(motorControlPosition, 0)
        dynamic[dataKey]['rightEngine'] = self.saveDynamicTableItem(motorControlPosition, 1)
        # dynamic[dataKey]['angle'] = self.saveDynamicTableItem(motorControlPositionInTableEnum, 2)


    def saveTwistControlsForKeyCombination(self, motorControlPosition, dynamicTwist):
        dataKey = motorControlPositionToDataKeyMap[motorControlPosition]

        dynamicTwist[dataKey] = {}
        dynamicTwist[dataKey]['linear'] = self.saveDynamicTwistTableItem(motorControlPosition, 0)
        dynamicTwist[dataKey]['angle'] = self.saveDynamicTwistTableItem(motorControlPosition, 1)

    def saveDynamicTwistTableItem(self, motorControlPosition, index):
        return self.widget.dynamicTwistTableUI.item(motorControlPosition, index).text()

    def saveDynamicTableItem(self, motorControlPosition, index):
        return self.widget.dynamicTable.item(motorControlPosition, index).text()
