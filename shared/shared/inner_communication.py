# from PyQt5.QtCore import pyqtSignal, QObject
from python_qt_binding.QtCore import pyqtSignal, QObject



class InnerCommunication(QObject):
    addRobotSignal = pyqtSignal(name="addedRobotSignal")
    deleteRobotSignal = pyqtSignal(object, name="deleteRobotSignal")
    closeApp = pyqtSignal()

    def test(self):
        print("InnerCommunication")

innerCommunication = InnerCommunication()