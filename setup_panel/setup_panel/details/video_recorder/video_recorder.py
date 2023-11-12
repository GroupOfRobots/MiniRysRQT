from python_qt_binding.QtWidgets import QWidget


class VideoRecorder(QWidget):
    def __init__(self, widget):
        super(VideoRecorder, self).__init__()
        self.widget = widget
        data = self.widget.data
        self.setData(data)

    def setData(self, data):
        videoRecorder = data.get("videoRecorder", {})

        self.widget.videoRecorderOutputLineEditUI.setText(videoRecorder.get('output', ''))

        quality = videoRecorder.get("quality")
        index = self.widget.videoRecorderQualityComboBoxUI.findText(quality)
        self.widget.videoRecorderQualityComboBoxUI.setCurrentIndex(index)

        config = videoRecorder.get("config", {})

        self.widget.videoRecorderConfigWidthSpinBoxUI.setValue(config.get("width", 640))
        self.widget.videoRecorderConfigHeightSpinBoxUI.setValue(config.get("height", 480))
        pass

    def saveData(self):
        self.widget.data['videoRecorder']["output"] = self.widget.videoRecorderOutputLineEditUI.text()
        self.widget.data['videoRecorder']["quality"] = self.widget.videoRecorderQualityComboBoxUI.currentText()
        self.widget.data['videoRecorder']["config"]["width"] = self.widget.videoRecorderConfigWidthSpinBoxUI.value()
        self.widget.data['videoRecorder']["config"]["width"] = self.widget.videoRecorderConfigHeightSpinBoxUI.value()
