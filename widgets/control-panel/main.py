# This Python file uses the following encoding: utf-8
import sys
from re import match
from unittest import case

from PySide6 import QtCore, QtWidgets, QtUiTools
from PySide6.QtGui import QPainter, QBrush
from PySide6.QtWidgets import QApplication, QWidget, QPushButton, QStyleOptionButton, QStyle, QStylePainter, \
    QSizePolicy, QMainWindow
from PySide6.QtCore import QFile, QSize
from PySide6.QtUiTools import QUiLoader
import sys
import os

# https://coderedirect.com/questions/167455/pyside2-qmainwindow-loaded-from-ui-file-not-triggering-window-events

class TestWindow(QtWidgets.QMainWindow):
    buttonBaseStyle='border: none;'
    keyPressedStyle='background-color: rgb(100, 200, 200);'
    buttonPressedStyle='background-color: rgb(100, 100, 100);'
    buttonIdleStyle= 'background-color: rgb(200, 200, 200);'

    def __init__(self, parent=None):
        super(TestWindow, self).__init__(parent)
        loader = QtUiTools.QUiLoader()
        file = QtCore.QFile(os.path.abspath("mainwindow.ui"))
        if file.open(QtCore.QFile.ReadOnly):
            self.window = loader.load(file, parent)
            file.close()
            self.setCentralWidget(self.window)

            self.defineButtons()
            self.addPressEventToButtons()

            self.show()

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return
        key=event.key()

        if key == QtCore.Qt.Key_W:
            self.setKeyPressedButtonColor(self.forwardButton)
        elif event.key() == QtCore.Qt.Key_D:
            self.setKeyPressedButtonColor(self.rightButton)
        elif event.key() == QtCore.Qt.Key_S:
            self.setKeyPressedButtonColor(self.backwardButton)
        elif event.key() == QtCore.Qt.Key_A:
            self.setKeyPressedButtonColor(self.leftButton)
        event.accept()

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return
        key=event.key()
        if key == QtCore.Qt.Key_W:
            self.setKeyReleasedButtonColor(self.forwardButton)
        elif event.key() == QtCore.Qt.Key_D:
            self.setKeyReleasedButtonColor(self.rightButton)
        elif event.key() == QtCore.Qt.Key_S:
            self.setKeyReleasedButtonColor(self.backwardButton)
        elif event.key() == QtCore.Qt.Key_A:
            self.setKeyReleasedButtonColor(self.leftButton)
        event.accept()

    def defineButtons(self):
        self.forwardButton = self.window.findChild(QPushButton, 'forwardButton')
        self.rightButton = self.window.findChild(QPushButton, 'rightButton')
        self.backwardButton = self.window.findChild(QPushButton, 'backwardButton')
        self.leftButton = self.window.findChild(QPushButton, 'leftButton')

    def addPressEventToButtons(self):
        self.forwardButton.pressed.connect(self.forwardButtonPressed)
        self.forwardButton.released.connect(self.forwardButtonReleased)

        self.rightButton.pressed.connect(self.rightButtonPressed)
        self.rightButton.released.connect(self.rightButtonReleased)

        self.backwardButton.pressed.connect(self.backwardButtonPressed)
        self.backwardButton.released.connect(self.backwardButtonReleased)

        self.leftButton.pressed.connect(self.leftButtonPressed)
        self.leftButton.released.connect(self.leftButtonReleased)

    def setPressedButtonColor(self, button):
        button.setStyleSheet(TestWindow.buttonBaseStyle+TestWindow.buttonPressedStyle)

    def setReleasedButtonColor(self, button):
        button.setStyleSheet(TestWindow.buttonBaseStyle + TestWindow.buttonIdleStyle)

    def setKeyPressedButtonColor(self, button):
        button.setStyleSheet(TestWindow.buttonBaseStyle+TestWindow.keyPressedStyle)

    def setKeyReleasedButtonColor(self, button):
        button.setStyleSheet(TestWindow.buttonBaseStyle + TestWindow.buttonIdleStyle)

    def forwardButtonPressed(self):
        self.setPressedButtonColor(self.forwardButton)

    def forwardButtonReleased(self):
        self.setReleasedButtonColor(self.forwardButton)

    def rightButtonPressed(self):
        self.setPressedButtonColor(self.rightButton)

    def rightButtonReleased(self):
        self.setReleasedButtonColor(self.rightButton)

    def backwardButtonPressed(self):
        self.setPressedButtonColor(self.backwardButton)

    def backwardButtonReleased(self):
        self.setReleasedButtonColor(self.backwardButton)

    def leftButtonPressed(self):
        self.setPressedButtonColor(self.leftButton)

    def leftButtonReleased(self):
        self.setReleasedButtonColor(self.leftButton)

    def setIconSize(self):
        print(self.backwardButton.size().width())
        print(self.backwardButton.size().height())
        width = self.backwardButton.size().width() * 0.9
        height = self.backwardButton.size().height() * 0.9
        self.forwardButton.setIconSize(QSize(width, height))
        self.leftButton.setIconSize(QSize(width, height))
        self.rightButton.setIconSize(QSize(width, height))
        self.backwardButton.setIconSize(QSize(width, height))

    def resizeEvent(self, event):
        print("resize")
        # print(self.backwardButton.size().width())
        self.setIconSize()

if __name__ == '__main__':
    import sys
    app = QtWidgets.QApplication(sys.argv)
    test = TestWindow()
    sys.exit(app.exec())