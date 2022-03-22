# This Python file uses the following encoding: utf-8

import os

from shared.base_widget.base_widget import BaseWidget

from std_msgs.msg import Float32
from ament_index_python import get_resource
from python_qt_binding import loadUi


class FanPanelWidget(BaseWidget):
    def __init__(self, stack=None, node=None):
        super(FanPanelWidget, self).__init__()
        BaseWidget.__init__(self, stack)

        self.loadUI()
        self.initializeRobotsOptions()

        self.publisher = node.create_publisher(Float32, '/internal/fan_output', 10)
        self.msg = Float32()
        self.value = 0

        self.fanSlider.setRange(0, 100)
        self.fanSlider.sliderReleased.connect(self.sliderReleased)
        self.fanSlider.valueChanged.connect(self.sliderValueChanged)

    def sliderReleased(self):
        self.msg.data = float(self.value)
        self.publisher.publish(self.msg)

    def sliderValueChanged(self, event):
        self.fanLcd.display(event)
        self.value = event / 100

    def loadUI(self):
        _, packagePath = get_resource('packages', 'fan_panel')
        uiFile = os.path.join(packagePath, 'share', 'fan_panel', 'resource', 'fan_panel.ui')
        loadUi(uiFile, self)
