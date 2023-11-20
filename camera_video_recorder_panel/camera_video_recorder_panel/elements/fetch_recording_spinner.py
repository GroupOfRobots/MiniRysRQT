from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QColor
from shared.spinner.spinner import Spinner


class FetchRecordingSpinner(Spinner):
    def __init__(self, widgetUI):
        super().__init__(parent=widgetUI,
                         centerOnParent=True,
                         disableParentWhenSpinning=False,
                         modality=Qt.WindowModality.NonModal,
                         roundness=50.0,
                         trailFadePercentage=80.0,
                         innerRadius=20,
                         numberOfLines=20,
                         lineLength=10,
                         lineWidth=20,
                         revolutionsPerSecond=0.5,
                         color=QColor(200, 50, 100)
                         )
