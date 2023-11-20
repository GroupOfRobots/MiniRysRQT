from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QColor
from shared.spinner.spinner import Spinner


class RecordingSpinner(Spinner):
    def __init__(self, widgetUI):
        super().__init__(parent=widgetUI,
                         centerOnParent=True,
                         disableParentWhenSpinning=False,
                         modality=Qt.WindowModality.NonModal,
                         roundness=40.0,
                         trailFadePercentage=90.0,
                         innerRadius=20,
                         numberOfLines=6,
                         lineLength=20,
                         lineWidth=20,
                         revolutionsPerSecond=0.5,
                         color=QColor(100, 200, 100))
