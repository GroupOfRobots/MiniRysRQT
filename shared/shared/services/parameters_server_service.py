from threading import Event

from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from shared.alert.alert import Alert


class ParametersServerService:
    def __init__(self, widget):
        self.widget = widget
        self.setParamsClient = None

    def setup(self, pidData, namespace):
        self.pidData = pidData
        self.namespace =namespace
        self.robotNode = self.findRobotNode(namespace)
        if self.robotNode:
            self.setParamsClients()
            return True
        return False

    def findRobotNode(self, robotNamespace):
        namesAndNamespaces = self.widget.node.get_node_names_and_namespaces()
        if len(namesAndNamespaces) == 0:
            return

        correctNamespacesElement = [(name, namespace) for name, namespace in namesAndNamespaces if
                                    robotNamespace in namespace]

        if len(correctNamespacesElement) == 0:
            return

        nodes = [(name, namespace) for name, namespace in namesAndNamespaces if 'motors_controller_cs' in name]
        if len(nodes) == 0:
            return
        node = nodes[0]
        return node

    def setParamsClients(self):
        nodeName = self.robotNode[0]
        nodeNamespace = self.robotNode[1]
        if nodeNamespace == '/':
            self.getParamsClient = self.widget.node.create_client(
                GetParameters, f'{nodeName}/get_parameters'.format_map(locals())
            )
            self.setParamsClient = self.widget.node.create_client(
                SetParameters, f'{nodeName}/set_parameters'.format_map(locals())
            )
        else:
            self.getParamsClient = self.widget.node.create_client(
                GetParameters, f'{nodeNamespace}/{nodeName}/get_parameters'.format_map(locals())
            )
            self.setParamsClient = self.widget.node.create_client(
                SetParameters, f'{nodeNamespace}/{nodeName}/set_parameters'.format_map(locals())
            )

    def callService(self, client, request, timeout=1.0):
        if not client.service_is_ready() and not client.wait_for_service(timeout):
            exceptionToDisplay = "Service timeout"
            Alert(self,"Setup widget", exceptionToDisplay)
            return

        # It is possible that a node has the parameter services but is not
        # spinning. In that is the case, the client call will time out.
        event = Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: event.set())

        event.wait(timeout)

        result = future.result()
        if result is None:
            exceptionToDisplay = "Service result was None"
            Alert(self,"PID widget: ", exceptionToDisplay)
            return

        return future.result()

    def setParameters(self):
        if self.setParamsClient is None:
            isSet = self.setup(self.pidData, self.namespace)
            if isSet is False:
                exceptionToDisplay = "Robot params setup is not available"
                Alert(self,"Parameters server", exceptionToDisplay)
            return

        pidSpeedKpParam = Parameter('pidSpeedKp', Parameter.Type.DOUBLE, self.pidData.get('pidAngleKp', 0))
        pidSpeedTiParam = Parameter('pidSpeedTi', Parameter.Type.DOUBLE, self.pidData.get('pidAngleTi', 0))
        pidSpeedTdParam = Parameter('pidSpeedTd', Parameter.Type.DOUBLE, self.pidData.get('pidAngleTd', 0))

        pidAngleKpParam = Parameter('pidAngleKp', Parameter.Type.DOUBLE, self.pidData.get('pidSpeedKp', 0))
        pidAngleTiParam = Parameter('pidAngleTi', Parameter.Type.DOUBLE, self.pidData.get('pidSpeedTi', 0))
        pidAngleTdParam = Parameter('pidAngleTd', Parameter.Type.DOUBLE, self.pidData.get('pidSpeedTd', 0))

        parameters = [pidSpeedKpParam,
                      pidSpeedTiParam,
                      pidSpeedTdParam,
                      pidAngleKpParam,
                      pidAngleTiParam,
                      pidAngleTdParam]

        setParametersRequest = SetParameters.Request()
        setParametersRequest.parameters = [p.to_parameter_msg() for p in parameters]

        response = self.callService(self.setParamsClient, setParametersRequest)
