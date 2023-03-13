from python_qt_binding.QtWidgets import QWidget, QTableWidgetItem
from .control_key_validator import ControlKeyValidator
from .controls_data_initializer import ControlsDataInitializer
from .controls_data_saver import ControlsDataSaver


class Controls(QWidget):
    def __init__(self, widget):
        super(Controls, self).__init__()
        self.widget = widget
        self.data = self.widget.data

        self.controlKeys = self.data.get('controlKeys', {})

        self.controlKeyValidator = ControlKeyValidator(self.widget, self.controlKeys)
        self.controlsDataInitializer = ControlsDataInitializer(self.widget, self.controlKeys, self.data)
        self.controlsDataSaver = ControlsDataSaver(self.widget, self.controlKeys, self.data)
