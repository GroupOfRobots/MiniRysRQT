# This Python file uses the following encoding: utf-8

import os
import sys

from PySide6 import QtCore, QtWidgets, QtUiTools, QtGui
from PySide6.QtCore import QSize, QFile, Qt, QPoint
from PySide6.QtGui import QPainter, QBrush, QPen
from PySide6.QtUiTools import QUiLoader
from PySide6.QtWidgets import QPushButton, QMainWindow, QWidget
# from ..elements.button import Button

# https://coderedirect.com/questions/167455/pyside2-qmainwindow-loaded-from-ui-file-not-triggering-window-events

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

            self.show()

    def paintEvent(self, event):
        # print("paint")
        painter = QPainter(self)
        painter.setPen(QPen(Qt.black, 5, Qt.SolidLine))

        painter.setBrush(QBrush(Qt.red, Qt.SolidPattern))
        width=self.width()
        height=self.height()
        painter.drawEllipse(width*0.05, height*0.05, 0.9*width, 0.9*height)
        painter.setBrush(QBrush(Qt.cyan, Qt.SolidPattern))
        painter.drawEllipse(self.joystickPosition.x(),self.joystickPosition.y(),10,10)


    def mouseMoveEvent(self, event):
        self.joystickPosition=event.position()
        if 0 < self.joystickPosition.x() < self.width() and 0 < self.joystickPosition.y() < self.height():
                # print('aaaaaaaaaaaaa')
            self.update()
        # self.update()


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
    mainWindow = Joystick()
    sys.exit(app.exec())
