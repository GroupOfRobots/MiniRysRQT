from shared.publishers.bool_publisher import BoolPublisher
from shared.services.parameters_server_service import ParametersServerService


class BalancePublisher:
    def __init__(self, checkBoxUI, node, widget):
        self.checkBoxUI = checkBoxUI
        self.node = node
        self.parametersServerService = ParametersServerService(widget)

        self.boolPublisher = BoolPublisher(self.checkBoxUI, self.node)
        self.checkBoxUI.stateChanged.connect(self.stateChanged)

    def setup(self, namespace, topic, pidData):
        self.boolPublisher.setTopic(namespace, topic)
        self.parametersServerService.setup(pidData, namespace)

    def stateChanged(self, state):
        if state == 0:
            return
        self.parametersServerService.setParameters()
