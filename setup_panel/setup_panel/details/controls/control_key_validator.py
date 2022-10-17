from shared.enums import ControlKeyEnum


class ControlKeyValidator:
    def __init__(self, widget, controlKeys):
        self.widget = widget
        self.controlKeys = controlKeys

        self.keyInputDictionary = {
            ControlKeyEnum.FORWARD: self.widget.forwardKeyInput,
            ControlKeyEnum.RIGHT: self.widget.rightKeyInput,
            ControlKeyEnum.BACKWARD: self.widget.backwardKeyInput,
            ControlKeyEnum.LEFT: self.widget.leftKeyInput,
            ControlKeyEnum.STABLE: self.widget.stableKeyInput,
        }

        self.addControlKeysValidators()

    def validator(self, event, key):
        button = self.keyInputDictionary[key]
        newValue = event.upper()
        button.setText(newValue)
        self.controlKeys[key] = newValue

        disableSaveButton = False
        for _, buttonKeys in self.createValuesMapItems():
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

    def createValuesMapItems(self):
        valuesMap = {}
        for key, value in self.controlKeys.items():
            if value in valuesMap:
                valuesMap[value].append(key)
            else:
                valuesMap[value] = [key]
        return valuesMap.items()

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