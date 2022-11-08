from python_qt_binding.QtWidgets import QWidget
from std_msgs.msg import Bool

class BalancePublisher(QWidget):
    def __init__(self, balanceCheckBoxUI, node):
        super(QWidget, self).__init__()
        self.balanceCheckBoxUI = balanceCheckBoxUI
        self.node= node
        self.balanceCheckBoxUI.stateChanged.connect(self.balanceStateChanged)

    def setTopic(self, namespace):
        self.balancePublisher = self.node.create_publisher(Bool, namespace + '/balance_mode', 10)


    def balanceStateChanged(self, state):
        msg = Bool()

        if state == 0:
            msg.data = False
            self.balancePublisher.publish(msg)
            return
        msg.data = True
        self.balancePublisher.publish(msg)

