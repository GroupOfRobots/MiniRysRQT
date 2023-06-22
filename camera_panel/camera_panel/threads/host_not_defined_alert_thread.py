from python_qt_binding.QtCore import QThread
from python_qt_binding.QtWidgets import QMessageBox


class HostNotDefinedAlertThread(QThread):
    def run(self):
        msg = QMessageBox()
        msg.setWindowTitle("Alert")
        msg.setText("This is an alert message")
        msg.setIcon(QMessageBox.Warning)
        msg.exec_()
