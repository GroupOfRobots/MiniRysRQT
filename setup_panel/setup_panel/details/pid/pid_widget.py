# from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import  QObject
from python_qt_binding.QtWidgets import QWidget

class PidWidget(QWidget):
    def __init__(self, widget):
        super(PidWidget, self).__init__()
        self.widget = widget
        data = self.widget.data

        # print(data)
        # print( data['dynamic'])
        # print(data)
        # print("test1")
        # print(data)
        self.setPidAngle(data['pid']['pidAngle'])
        self.setPidSpeed(data['pid']['pidSpeed'])


    def connectElements(self):
        self.widget.pidAngleKpUI.valueChanged.connect(self.test)
        self.widget.pidAngleKiUI.valueChanged.connect(self.test)
        self.widget.pidAngleKdUI.valueChanged.connect(self.test)

        self.widget.pidSpeedKpUI.valueChanged.connect(self.test)
        self.widget.pidSpeedKiUI.valueChanged.connect(self.test)
        self.widget.pidSpeedKdUI.valueChanged.connect(self.test)

    def setPidAngle(self, pidAngleData):
        self.widget.pidAngleKpUI.setValue(pidAngleData['Kp'])
        self.widget.pidAngleKiUI.setValue(pidAngleData['Ki'])
        self.widget.pidAngleKdUI.setValue(pidAngleData['Kd'])

    def setPidSpeed(self, pidSpeedData):
        self.widget.pidSpeedKpUI.setValue(pidSpeedData['Kp'])
        self.widget.pidSpeedKiUI.setValue(pidSpeedData['Ki'])
        self.widget.pidSpeedKdUI.setValue(pidSpeedData['Kd'])

    def test(self, event):
        print("event")
        print(event)
