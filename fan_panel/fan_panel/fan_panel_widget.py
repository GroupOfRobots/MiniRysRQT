# This Python file uses the following encoding: utf-8

from shared.base_widget.base_widget import BaseWidget
from shared.enums import PackageNameEnum
from shared.inner_communication import innerCommunication
from std_msgs.msg import Float32


class FanPanelWidget(BaseWidget):
    activeRobotsMap = {}

    def __init__(self, stack=None, node=None, mainPanel=None):
        super(FanPanelWidget, self).__init__(stack, PackageNameEnum.FanPanel)

        self.fanPanel = mainPanel
        self.id = None
        self.setRobotOnScreen()

        self.publisher = node.create_publisher(Float32, '/internal/fan_output', 10)
        self.msg = Float32()
        self.value = 0

        self.setupFanSlider()

        self.spinBox.valueChanged.connect(self.fanSpinBoxValueChanged)
        self.movedBySlider = False
        self.movedBySpinBox = False

        self.fanPanel.closePanelSignal.connect(self.onClosePanelSignal)

        innerCommunication.updateFanValueSignal.connect(self.onUpdateValueSignal)

    def setRobotOnScreen(self):
        self.checkIfRobotExists()
        super().setRobotOnScreen()

        self.id = self.data.get('id')
        if FanPanelWidget.activeRobotsMap.get(self.id) is None:
            FanPanelWidget.activeRobotsMap[self.id] = 1
        else:
            FanPanelWidget.activeRobotsMap[self.id] += 1

    def onDeleteRobotSignal(self, signalData):
        self.checkIfRobotExists()
        super().onDeleteRobotSignal(signalData)

    def checkIfRobotExists(self):
        if self.id is not None:
            FanPanelWidget.activeRobotsMap[self.id] -= 1
            if FanPanelWidget.activeRobotsMap[self.id] == 0:
                self.turnOffFan()

    def setupFanSlider(self):
        self.fanSlider.sliderReleased.connect(self.sliderReleased)
        self.fanSlider.valueChanged.connect(self.sliderValueChanged)

    def sliderReleased(self):
        self.sendFanValue()
        self.updateFans()

    def updateFans(self):
        fanData = {
            "panelName": self.fanPanel.name,
            "id": self.data.get('id'),
            "value": self.value
        }

        innerCommunication.updateFanValueSignal.emit(fanData)

    def sendFanValue(self):
        self.msg.data = float(self.value)
        self.publisher.publish(self.msg)

    def fanSpinBoxValueChanged(self, event):
        self.movedBySpinBox = True

        if self.movedBySlider:
            self.movedBySlider = False
        else:
            self.fanSlider.setValue(event)
            self.value = event / 100
            self.sendFanValue()
            self.updateFans()

    def sliderValueChanged(self, event):
        self.movedBySlider = True
        if self.movedBySpinBox:
            self.movedBySpinBox = False
        else:
            self.spinBox.setValue(event)
            self.value = event / 100

    def turnOffFan(self):
        self.value = 0
        self.msg.data = float(self.value)
        self.publisher.publish(self.msg)

    def onClosePanelSignal(self):
        self.checkIfRobotExists()
        self.turnOffFan()

    def onUpdateValueSignal(self, event):
        panelName = event.get('panelName')
        eventId = event.get('id')
        currentId = self.data.get('id')
        if panelName != self.fanPanel.name and eventId == currentId:
            newValue = event.get('value')
            self.value = newValue
            valueToDisplay = int(newValue * 100)
            self.spinBox.setValue(valueToDisplay)
            self.fanSlider.setValue(valueToDisplay)
