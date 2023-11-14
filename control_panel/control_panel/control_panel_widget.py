# This Python file uses the following encoding: utf-8

from python_qt_binding.QtCore import Qt
from shared.base_widget.base_widget import BaseWidget
from shared.bool_publisher.bool_publisher import BoolPublisher
from shared.enums import ControlKeyEnum, PackageNameEnum

from .elements.button import Button
from .elements.key_state import getKeyState
from .elements.message_service import MessageService


class ControlPanelWidget(BaseWidget):
    def __init__(self, stack=None, node=None):
        super(ControlPanelWidget, self).__init__(stack, PackageNameEnum.ControlPanel, node=node)

        self.balancePublisher = BoolPublisher(self.balanceCheckBoxUI, self.node)
        self.servoPublisher = BoolPublisher(self.servoCheckBoxUI, self.node)
        self.messageService = MessageService(self.messageTypeComboBoxUI, self.node)

        self.setRobotOnScreen()

        self.defineButtons()

        self.initPressedKeys()

    def initPressedKeys(self):
        self.pressedKeys = {
            ControlKeyEnum.FORWARD: False,
            ControlKeyEnum.RIGHT: False,
            ControlKeyEnum.BACKWARD: False,
            ControlKeyEnum.LEFT: False
        }

    def initializeRobotSettings(self):
        self.controlKeys = self.data.get('controlKeys', {})

        self.balancePublisher.setTopic(self.namespace, '/balance_mode')
        self.servoPublisher.setTopic(self.namespace, '/servo_status')
        self.messageService.setup(self.namespace, self.data)

        for key in self.controlKeys:
            controlValue = self.controlKeys[key].upper()
            self.controlKeys[key] = Qt.Key(ord(controlValue))

    def keyPressEvent(self, event):
        self.keyEvent(event, True)

    def keyReleaseEvent(self, event):
        self.keyEvent(event, False)

    def keyEvent(self, event, isButtonPressed):
        if event.isAutoRepeat():
            return
        button = None
        key = event.key()

        if key == self.controlKeys[ControlKeyEnum.FORWARD]:
            self.pressedKeys[ControlKeyEnum.FORWARD] = isButtonPressed
            button = self.forwardButtonElement
        elif key == self.controlKeys[ControlKeyEnum.RIGHT]:
            self.pressedKeys[ControlKeyEnum.RIGHT] = isButtonPressed
            button = self.rightButtonElement
        elif key == self.controlKeys[ControlKeyEnum.BACKWARD]:
            self.pressedKeys[ControlKeyEnum.BACKWARD] = isButtonPressed
            button = self.backwardButtonElement
        elif key == self.controlKeys[ControlKeyEnum.LEFT]:
            self.pressedKeys[ControlKeyEnum.LEFT] = isButtonPressed
            button = self.leftButtonElement

        if button:
            self.sendMessageOnKeyStateBase()
            if isButtonPressed:
                button.pressedKeyState()
            else:
                button.releasedKeyState()
        event.accept()

    def sendMessageOnKeyStateBase(self):
        keyState = getKeyState(self.pressedKeys)

        msg = self.messageService.messageFunction(keyState)

        if msg is not None:
            self.messageService.publisher.publish(msg)

    def buttonClicked(self, isPressed, controlKeyEnum):
        self.pressedKeys[controlKeyEnum] = isPressed
        self.sendMessageOnKeyStateBase()

    def defineButtons(self):
        self.settingsButton.clicked.connect(self.settingsClicked)

        self.forwardButtonElement = Button(self.forwardButton)
        self.rightButtonElement = Button(self.rightButton)
        self.backwardButtonElement = Button(self.backwardButton)
        self.leftButtonElement = Button(self.leftButton)

        self.forwardButton.pressed.connect(lambda: self.buttonClicked(True, ControlKeyEnum.FORWARD))
        self.rightButton.pressed.connect(lambda: self.buttonClicked(True, ControlKeyEnum.RIGHT))
        self.backwardButton.pressed.connect(lambda: self.buttonClicked(True, ControlKeyEnum.BACKWARD))
        self.leftButton.pressed.connect(lambda: self.buttonClicked(True, ControlKeyEnum.LEFT))

        self.forwardButton.released.connect(lambda: self.buttonClicked(False, ControlKeyEnum.FORWARD))
        self.rightButton.released.connect(lambda: self.buttonClicked(False, ControlKeyEnum.RIGHT))
        self.backwardButton.released.connect(lambda: self.buttonClicked(False, ControlKeyEnum.BACKWARD))
        self.leftButton.released.connect(lambda: self.buttonClicked(False, ControlKeyEnum.LEFT))

    def resizeEvent(self, event):
        self.setIconSize()

    def setIconSize(self):
        width = int(self.backwardButton.size().width() * 0.9)
        height = int(self.backwardButton.size().height() * 0.9)

        self.forwardButtonElement.resizeIcon(width, height)
        self.leftButtonElement.resizeIcon(width, height)
        self.rightButtonElement.resizeIcon(width, height)
        self.backwardButtonElement.resizeIcon(width, height)
