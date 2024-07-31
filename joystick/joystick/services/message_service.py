from geometry_msgs.msg import Twist
from minirys_msgs.msg import MotorCommand
from python_qt_binding.QtWidgets import QWidget


class MessageService(QWidget):
    def __init__(self, messageTypeComboBoxUI, node):
        super(QWidget, self).__init__()
        self.messageTypeComboBoxUI = messageTypeComboBoxUI
        self.node = node
        self.messageTypeComboBoxUI.currentIndexChanged.connect(self.setMessageFunctionAndPublisher)

    def setup(self, namespace):
        self.namespace = namespace
        self.setMessageFunctionAndPublisher(self.messageTypeComboBoxUI.currentIndex())

    def setMessageFunctionAndPublisher(self, index):
        if index == 0:
            self.publisher = self.node.create_publisher(Twist, self.namespace + '/cmd_vel', 10)

        elif index == 1:
            self.publisher = self.node.create_publisher(MotorCommand, self.namespace + '/internal/motor_command', 10)

    def setupTwistMessage(self, linear, angular):
        msg = Twist()

        msg.linear.x = linear
        msg.angular.z = angular

        return msg

    def setupMotorCommandMessage(self, leftEngine, rightEngine):
        msg = MotorCommand()

        msg.speed_l = leftEngine
        msg.speed_r = rightEngine

        return msg

    def publishTwist(self, linear, angular):
        msg = self.setupTwistMessage(linear, angular)
        self.publisher.publish(msg)


    def publishMotorCommand(self, leftEngine, rightEngine):
        msg = self.setupMotorCommandMessage( leftEngine, rightEngine)
        self.publisher.publish(msg)
