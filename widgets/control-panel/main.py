# This Python file uses the following encoding: utf-8

import os
import sys

from PySide6 import QtCore, QtWidgets, QtUiTools
from PySide6.QtCore import QSize, QFile
from PySide6.QtUiTools import QUiLoader
from PySide6.QtWidgets import QPushButton, QMainWindow
from elements.button import Button


# https://coderedirect.com/questions/167455/pyside2-qmainwindow-loaded-from-ui-file-not-triggering-window-events

class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        loader = QUiLoader()
        file = QFile(os.path.abspath("mainwindow.ui"))
        if file.open(QFile.ReadOnly):
            self.window = loader.load(file, parent)
            file.close()
            self.setCentralWidget(self.window)

            self.defineButtons()

            self.show()

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

    def defineButtons(self):
        self.forwardButton = Button(self.window.findChild(QPushButton, 'forwardButton'))
        self.rightButton = Button(self.window.findChild(QPushButton, 'rightButton'))
        self.backwardButton = Button(self.window.findChild(QPushButton, 'backwardButton'))
        self.leftButton = Button(self.window.findChild(QPushButton, 'leftButton'))

    def resizeEvent(self, event):
        print("resize")
        # print(self.backwardButton.size().width())
        self.setIconSize()

    def setIconSize(self):
        width = self.backwardButton.size().width() * 0.9
        height = self.backwardButton.size().height() * 0.9
        self.forwardButton.resizeIcon(width,height)
        self.leftButton.resizeIcon(width,height)
        self.rightButton.resizeIcon(width,height)
        self.backwardButton.resizeIcon(width,height)


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    mainWindow = MainWindow()
    sys.exit(app.exec())
