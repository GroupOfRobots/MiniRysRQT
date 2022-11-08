from python_qt_binding.QtWidgets import QWidget

from geometry_msgs.msg import Twist
from minirys_msgs.msg import MotorCommand

from .key_state import KeyStateMap


class MessageService(QWidget):
    def __init__(self, messageTypeComboBoxUI, node):
        super(QWidget, self).__init__()
        self.messageTypeComboBoxUI = messageTypeComboBoxUI
        self.node = node
        self.messageTypeComboBoxUI.currentIndexChanged.connect(self.setMessageFunctionAndPublisher)

    def setup(self, namespace,  data):
        self.namespace = namespace
        self.dynamic = data.get('dynamic', {})
        self.dynamicTwist = data.get('dynamicTwist', {})
        self.setMessageFunctionAndPublisher(self.messageTypeComboBoxUI.currentIndex())

    def setMessageFunctionAndPublisher(self, index):
        if index == 0:
            self.messageFunction = self.setupTwistMessage
            self.publisher = self.node.create_publisher(Twist, self.namespace + '/cmd_vel', 10)

        elif index == 1:
            self.messageFunction = self.setupMotorCommandMessage
            self.publisher = self.node.create_publisher(MotorCommand, self.namespace + '/internal/motor_command', 10)

    def setupTwistMessage(self, keyState):
        msg = Twist()

        dataKey = KeyStateMap.get(keyState, None)

        if dataKey is not None:
            msg.linear.y = float(self.dynamicTwist[dataKey]['linear'])
            msg.angular.z = float(self.dynamicTwist[dataKey]['angle'])
        elif not (keyState.forward or keyState.right or keyState.backward or keyState.left):
            msg.linear.y = 0.0
            msg.angular.z = 0.0
        elif dataKey is None:
            return
        return msg

    def setupMotorCommandMessage(self, keyState):
        msg = MotorCommand()

        dataKey = KeyStateMap.get(keyState)

        if dataKey is not None:
            msg.speed_l = float(self.dynamic[dataKey]['leftEngine'])
            msg.speed_r = float(self.dynamic[dataKey]['rightEngine'])
        elif not (keyState.forward or keyState.right or keyState.backward or keyState.left):
            msg.speed_l = 0.0
            msg.speed_r = 0.0
        elif dataKey is None:
            return
        return msg
