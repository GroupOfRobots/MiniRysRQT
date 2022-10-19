from shared.enums import ControlKeyEnum
from shared.enums import MotorControlPositionInTableEnum, motorControlPositionToDataKeyMap

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

        for motorControlPositionInTable in MotorControlPositionInTableEnum:
            self.saveControlsForKeyCombination(motorControlPositionInTable, dynamic)
            self.saveTwistControlsForKeyCombination(motorControlPositionInTable, dynamicTwist)

        self.data['dynamic']= dynamic
        self.data['dynamicTwist'] = dynamicTwist

    def saveControlsForKeyCombination(self, motorControlPositionInTableEnum, dynamic):
        dataKey = motorControlPositionToDataKeyMap[motorControlPositionInTableEnum]

        dynamic[dataKey] = {}
        dynamic[dataKey]['leftEngine'] = self.saveDynamicTableItem(motorControlPositionInTableEnum, 0)
        dynamic[dataKey]['rightEngine'] = self.saveDynamicTableItem(motorControlPositionInTableEnum, 1)
        # dynamic[dataKey]['angle'] = self.saveDynamicTableItem(motorControlPositionInTableEnum, 2)


    def saveTwistControlsForKeyCombination(self, motorControlPositionInTableEnum, dynamicTwist):
        dataKey = motorControlPositionToDataKeyMap[motorControlPositionInTableEnum]

        dynamicTwist[dataKey] = {}
        dynamicTwist[dataKey]['linear'] = self.saveDynamicTwistTableItem(motorControlPositionInTableEnum, 0)
        dynamicTwist[dataKey]['angle'] = self.saveDynamicTwistTableItem(motorControlPositionInTableEnum, 1)

    def saveDynamicTwistTableItem(self, motorControlPositionInTableEnum, index):
        return self.widget.dynamicTwistTableUI.item(motorControlPositionInTableEnum, index).text()

    def saveDynamicTableItem(self, motorControlPositionInTableEnum, index):
        return self.widget.dynamicTable.item(motorControlPositionInTableEnum, index).text()
