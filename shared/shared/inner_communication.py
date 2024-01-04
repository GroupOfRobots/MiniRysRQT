# from PyQt5.QtCore import pyqtSignal, QObject
from python_qt_binding.QtCore import pyqtSignal, QObject


class InnerCommunication(QObject):
    addRobotSignal = pyqtSignal(object, name="addedRobotSignal")
    deleteRobotSignal = pyqtSignal(object, name="deleteRobotSignal")
    updateRobotSignal = pyqtSignal(object, name="updateRobotSignal")
    showAlert = pyqtSignal(object, name="showAlert")
    closeApp = pyqtSignal()

    updateFanValueSignal = pyqtSignal(object, name="updateFanValue")
    lastFanValues = {}

    def test(self):
        print("InnerCommunication")

innerCommunication = InnerCommunication()