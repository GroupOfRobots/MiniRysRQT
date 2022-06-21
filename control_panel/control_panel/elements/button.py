from python_qt_binding.QtCore import QSize
from python_qt_binding.QtWidgets import QPushButton


class Button(QPushButton):
    baseStyle= 'border: none;'
    keyPressedStyle='background-color: rgb(100, 200, 200);'
    pressedStyle= 'background-color: rgb(100, 100, 100);'
    idleStyle= 'background-color: rgb(200, 200, 200);'

    def __init__(self, button):
        super(Button, self).__init__()
        self.button = button
        self.button.pressed.connect(self.pressedState)
        self.button.released.connect(self.releasedState)

    def size(self):
        return self.button.size()

    def resizeIcon(self, width,height):
        self.button.setIconSize(QSize(width, height))

    def pressedKeyState(self):
        self.setKeyPressedColor()

    def releasedKeyState(self):
        self.setIdleStyle()

    def setKeyPressedColor(self):
        self.button.setStyleSheet(Button.baseStyle + Button.keyPressedStyle)

    def pressedState(self):
        self.setPressedColor()

    def releasedState(self):
        self.setIdleStyle()

    def setPressedColor(self):
        self.button.setStyleSheet(Button.baseStyle + Button.pressedStyle)

    def setIdleStyle(self):
        self.button.setStyleSheet(Button.baseStyle + Button.idleStyle)