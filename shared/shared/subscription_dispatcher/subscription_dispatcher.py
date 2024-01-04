from python_qt_binding.QtCore import QObject
from python_qt_binding.QtCore import pyqtSignal


class SubscriptionDispatcher(QObject):
    valueSignal = pyqtSignal(object)

    def __init__(self, node):
        super(SubscriptionDispatcher, self).__init__()
        print("SubscriptionDispatcher")
        self.node = node

    def __del__(self):
        # pass
        print('DashboardDistanceSensorsWidget Destructor called, Employee deleted.')

    def getSubscription(self, messageType, topic):
        return self.node.create_subscription(messageType, topic, self.valueCallback, 10)

    def valueCallback(self, event):
        self.valueSignal.emit(event)
