import threading

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import QMessageBox


class Alert:
    def __init__(self, widgetName, alertText):
        self.alertTitle = str("Alert: " + widgetName)
        self.alertText = str(alertText)
        self.commandThread = threading.Thread(target=self.run, args=(self.alertTitle, self.alertText))
        self.commandThread.start()

    def run(self,alertTitle,alertText):
        msg = QMessageBox()
        msg.setWindowTitle(alertTitle)
        msg.setText(alertText)
        msg.setIcon(QMessageBox.Warning)
        msg.setWindowModality(Qt.NonModal)

        msg.show()
        msg.exec_()
