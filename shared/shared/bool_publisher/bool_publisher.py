from python_qt_binding.QtWidgets import QWidget
from std_msgs.msg import Bool


class BoolPublisher(QWidget):
    def __init__(self, checkBoxUI, node):
        super(QWidget, self).__init__()
        self.checkBoxUI = checkBoxUI
        self.node = node
        self.checkBoxUI.stateChanged.connect(self.stateChanged)

    def setTopic(self, namespace, topic):
        self.publisher = self.node.create_publisher(Bool, namespace + topic, 10)

    def stateChanged(self, state):
        msg = Bool()

        if state == 0:
            msg.data = False
            self.publisher.publish(msg)
            return
        msg.data = True
        self.publisher.publish(msg)
