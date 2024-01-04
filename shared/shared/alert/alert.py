from python_qt_binding.QtCore import QObject
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import QDesktopWidget
from python_qt_binding.QtWidgets import QMessageBox
from shared.inner_communication import innerCommunication


class GlobalAlert(QObject):
    def __init__(self):
        super(GlobalAlert, self).__init__()
        innerCommunication.showAlert.connect(self.showAlert)

    def showAlert(self, event):
        alertTitle = event.get("widgetName", "")
        alertText = event.get("alertText", "")
        Alert(QDesktopWidget(), alertTitle, alertText)


globalAlert = GlobalAlert()


class Alert:
    def __init__(self, widget=None, widgetName=None, alertText=None):
        if widget is not None:
            alert = QMessageBox(widget)
        else:
            alert = QMessageBox()

        alert.setIcon(QMessageBox.Information)
        alertTitle = str("Alert: " + widgetName)
        alertText = str(alertText)
        alert.setText(alertText)
        alert.setText(alertText)
        alert.setWindowTitle(alertTitle)
        alert.setStandardButtons(QMessageBox.Ok)

        alert.setWindowModality(Qt.NonModal)  # Set the non-blocking flag

        centerX, centerY = self.getDesktopCenterPosition(alert)

        # Set the alert position to the center of the screen
        alert.move(centerX, centerY)
        alert.show()

    def getDesktopCenterPosition(self, alert):
        desktop = QDesktopWidget()
        screenGeometry = desktop.screenGeometry()

        # Calculate the center position of the screen
        centerX = screenGeometry.x() + (screenGeometry.width() - alert.width()) // 2
        centerY = screenGeometry.y() + (screenGeometry.height() - alert.height()) // 2
        return centerX, centerY
