from python_qt_binding.QtCore import QSize
from python_qt_binding.QtWidgets import QPushButton


class Button(QPushButton):
    baseStyle= 'border: none;'
    keyPressedStyle='background-color: rgb(100, 200, 200);'
    mousePressedStyle= 'background-color: rgb(100, 100, 100);'
    idleStyle= 'background-color: rgb(200, 200, 200);'

    def __init__(self, button):
        super(Button, self).__init__()
        self.button = button
        self.button.pressed.connect(self.mousePressedState)
        self.button.released.connect(self.mouseReleasedState)

        self.isKeyPressed=False
        self.isMousePressed=False


    def size(self):
        return self.button.size()

    def resizeIcon(self, width,height):
        self.button.setIconSize(QSize(width, height))

    def pressedKeyState(self):
        self.setKeyPressedColor()
        self.isKeyPressed=True

    def releasedKeyState(self):
        if self.isMousePressed:
            self.setPressedColor()
        else:
            self.setIdleStyle()
        self.isKeyPressed=False

    def setKeyPressedColor(self):
        self.button.setStyleSheet(Button.baseStyle + Button.keyPressedStyle)

    def mousePressedState(self):
        self.setPressedColor()
        self.isMousePressed=True

    def mouseReleasedState(self):
        if self.isKeyPressed:
            self.setKeyPressedColor()
        else:
            self.setIdleStyle()
        self.isMousePressed=False

    def setPressedColor(self):
        self.button.setStyleSheet(Button.baseStyle + Button.mousePressedStyle)

    def setIdleStyle(self):
        self.button.setStyleSheet(Button.baseStyle + Button.idleStyle)