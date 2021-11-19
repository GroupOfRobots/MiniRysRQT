# This Python file uses the following encoding: utf-8
import math
import os
import sys

from PySide6 import QtCore, QtWidgets, QtUiTools, QtGui
from PySide6.QtCore import QSize, QFile, Qt, QPoint
from PySide6.QtGui import QPainter, QBrush, QPen
from PySide6.QtUiTools import QUiLoader
from PySide6.QtWidgets import QPushButton, QMainWindow, QWidget


# from ..elements.button import Button

# https://coderedirect.com/questions/167455/pyside2-qmainwindow-loaded-from-ui-file-not-triggering-window-events


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.form_widget = Joystick(self)
        self.setCentralWidget(self.form_widget)
        self.show()

class Joystick(QWidget):
    def __init__(self, parent=None):
        super(Joystick, self).__init__(parent)
        loader = QUiLoader()
        file = QFile(os.path.abspath("joystick.ui"))
        if file.open(QFile.ReadOnly):
            self.joystickPosition=QPoint(0,0)
            self.window = loader.load(file, parent)
            file.close()
                      # self.setCentralWidget(self.window)

            # self.defineButtons()

            # self.show()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setPen(QPen(Qt.black, 5, Qt.SolidLine))

        self.paintJoystickBoundary(painter)
        self.paintJoystick(painter)

    def paintJoystickBoundary(self, painter):
        width = self.width()
        height = self.height()
        x = int(width * 0.05)
        y = int(height * 0.05)
        rx = int(0.9 * width)
        ry = int(0.9 * height)
        painter.setPen(QPen(Qt.black, 5, Qt.SolidLine))
        painter.setBrush(QBrush(Qt.red, Qt.SolidPattern))
        painter.drawEllipse(x, y, rx, ry)

    def paintJoystick(self, painter):
        width = self.width()
        height = self.height()
        rx = int(0.2 * width)
        ry = int(0.2 * height)
        x = int(self.joystickPosition.x() - 0.5 * rx)
        y = int(self.joystickPosition.y() - 0.5 * ry)
        painter.setBrush(QBrush(Qt.cyan, Qt.SolidPattern))
        painter.drawEllipse(x, y, rx, ry)

    def mouseReleaseEvent(self,event):
        self.joystickPosition = QPoint(self.width() * 0.5, self.height() * 0.5)
        self.update()


    def mouseMoveEvent(self, event):
        print("mouseMoveEvent")
        self.joystickPosition = event.position()
        print(self.joystickPosition)
        # if (self.width()*0.15) < self.joystickPosition.x() < (self.width()*0.85) and (self.height()*0.15) < self.joystickPosition.y() < (self.height()*0.85):
        x = self.joystickPosition.x()
        y = self.joystickPosition.y()
        h = self.width() * 0.5
        k = self.height() * 0.5
        rx = self.width() * 0.45 - 0.1 * self.width()
        ry = self.height() * 0.45 - 0.1 * self.height()
        # print("(x-h)**2/rx**2+(y-k)**2/ry**2")
        # print((x-h)**2/rx**2+(y-k)**2/ry**2)
        if ((x - h) ** 2 / rx ** 2 + (y - k) ** 2 / ry ** 2 <= 1):
            self.update()


    def resizeEvent(self, event):
        # print("event")
        self.joystickPosition = QPoint(self.width() * 0.5, self.height() * 0.5)

        # print(event.oldSize())
        # print(event.size())
        # if (event.oldSize().height() == -1):
        #     return
        # x = event.size().width() / event.oldSize().width() * self.joystickPosition.x()
        # y = event.size().height() / event.oldSize().height() * self.joystickPosition.y()
        # print(x, y)
        # self.joystickPosition.setX((x))
        # self.joystickPosition.setY((y))
        # print(self.joystickPosition)

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return
        key = event.key()
        if key == QtCore.Qt.Key_W:
            self.forwardButton.pressedKeyState()
        elif event.key() == QtCore.Qt.Key_D:
            self.rightButton.pressedKeyState()
        elif event.key() == QtCore.Qt.Key_S:
            self.backwardButton.pressedKeyState()
        elif event.key() == QtCore.Qt.Key_A:
            self.leftButton.pressedKeyState()
        event.accept()

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return
        key = event.key()
        if key == QtCore.Qt.Key_W:
            self.forwardButton.releasedKeyState()
        elif event.key() == QtCore.Qt.Key_D:
            self.rightButton.releasedKeyState()
        elif event.key() == QtCore.Qt.Key_S:
            self.backwardButton.releasedKeyState()
        elif event.key() == QtCore.Qt.Key_A:
            self.leftButton.releasedKeyState()
        event.accept()

    # def defineButtons(self):
    #     self.forwardButton = Button(self.window.findChild(QPushButton, 'forwardButton'))
    #     self.rightButton = Button(self.window.findChild(QPushButton, 'rightButton'))
    #     self.backwardButton = Button(self.window.findChild(QPushButton, 'backwardButton'))
    #     self.leftButton = Button(self.window.findChild(QPushButton, 'leftButton'))

    # def resizeEvent(self, event):
    #     print("resize1")
    #     # print(self.window.frameGeometry().width())
    #     # print(self.window.frameGeometry().height())
    #     mainWindow = QWidget()
    #     width = self.window.frameGeometry().width()
    #     height = mainWindow.frameGeometry().height()
    #     # print(self.height())
    #     # print(width)
    #     # print(height)
    #     # print(event)
    #     # print(self.window.frameGeometry.width(), self.window.frameGeometry.height())
    #     # print(self.backwardButton.size().width())
    #     # self.setIconSize()

    # def setIconSize(self):
    #     width = self.backwardButton.size().width() * 0.9
    #     height = self.backwardButton.size().height() * 0.9
    #     self.forwardButton.resizeIcon(width,height)
    #     self.leftButton.resizeIcon(width,height)
    #     self.rightButton.resizeIcon(width,height)
    #     self.backwardButton.resizeIcon(width,height)


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    # mainWindow = Joystick()
    mainWindow = MainWindow()
    sys.exit(app.exec())
