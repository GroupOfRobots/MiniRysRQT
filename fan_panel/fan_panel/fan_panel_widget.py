# This Python file uses the following encoding: utf-8

import os

from python_qt_binding.QtWidgets import QAbstractSpinBox
from shared.base_widget.base_widget import BaseWidget

from std_msgs.msg import Float32
from ament_index_python import get_resource
from python_qt_binding import loadUi

from shared.inner_communication import innerCommunication

from shared.enums import PackageNameEnum


class FanPanelWidget(BaseWidget):
    def __init__(self, stack=None, node=None, fanPanel=None):
        super(FanPanelWidget, self).__init__(stack, PackageNameEnum.FanPanel)

        self.fanPanel = fanPanel

        # self.loadUI()
        self.initializeRobotsOptions()

        self.publisher = node.create_publisher(Float32, '/internal/fan_output', 10)
        self.msg = Float32()
        self.value = 0

        self.fanSlider.setRange(0, 100)
        self.fanSlider.sliderReleased.connect(self.sliderReleased)
        self.fanSlider.valueChanged.connect(self.sliderValueChanged)

        self.spinBox.valueChanged.connect(self.fanSpinBoxValueChanged)
        self.movedBySlider = False
        self.movedBySpinBox = False

        self.fanPanel.closePanelSignal.connect(self.onClosePanelSignal)

        innerCommunication.updateFanValueSignal.connect(self.onUpdateValueSignal)

    def sliderReleased(self):
        self.sendFanValue()
        self.updateFans()

    def updateFans(self):
        fanData = {
            "panelName": self.fanPanel.name,
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

    # def loadUI(self):
    #     _, packagePath = get_resource('packages', 'fan_panel')
    #     uiFile = os.path.join(packagePath, 'share', 'fan_panel', 'resource', 'fan_panel.ui')
    #     loadUi(uiFile, self)

    def onClosePanelSignal(self):
        self.value = 0
        self.msg.data = float(self.value)
        self.publisher.publish(self.msg)

    def onUpdateValueSignal(self, event):
        panelName = event.get('panelName')
        if panelName != self.fanPanel.name:
            newValue = event.get('value')
            self.value = newValue
            valueToDisplay = int(newValue * 100)
            self.spinBox.setValue(valueToDisplay)
            self.fanSlider.setValue(valueToDisplay)
