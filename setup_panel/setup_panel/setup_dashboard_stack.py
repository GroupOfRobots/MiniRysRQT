# This Python file uses the following encoding: utf-8
import os, os.path



import asyncio
import can



from python_qt_binding.QtWidgets import QWidget, QStackedWidget, QHBoxLayout, QListWidget, QGridLayout, QLayout



# import QObjectClass
# import PyQt5.QtWidgets


from .dashboard_element import DashboardElementWidget
from .setup_widget import SetupWidget
from .setup_dashboard import SetupDashboardWidget


class SetupDashboardStackWidget(QWidget):
    def __init__(self, node, plugin=None, context=None):
        super(SetupDashboardStackWidget, self).__init__()

        self.context= context

        # self.setObjectName("aaa")

        self.stack = QStackedWidget(self)

        self.node =node

        self.controlPanelWidget = SetupDashboardWidget(node, plugin = self,context=context)

        self.stack.addWidget(self.controlPanelWidget)

        hbox = QGridLayout(self)
        hbox.addWidget(self.stack)

        self.stack.setCurrentIndex(0)

        self.setLayout(hbox)

    #     can0 = can.Bus('vcan0', bustype='virtual', receive_own_messages=False)
    #     reader = can.AsyncBufferedReader()
    #     logger = can.Logger('logfile.asc')
    #
    #     listeners = [
    #         self.print_message,  # Callback function
    #         reader,  # AsyncBufferedReader() listener
    #         logger  # Regular Listener object
    #     ]
    #     # Create Notifier with an explicit loop to use for scheduling of callbacks
    #     loop = asyncio.get_event_loop()
    #     notifier = can.Notifier(can0, listeners, loop=loop)
    #
    #     # Start sending first message
    #     can0.send(can.Message(arbitration_id=0))
    #
    #     print('Bouncing 10 messages...')
    #     for _ in range(10):
    #         # Wait for next message from AsyncBufferedReader
    #         msg =  await reader.get_message()
    #         # Delay response
    #         # await asyncio.sleep(0.5)
    #         # msg.arbitration_id += 1
    #         # can0.send(msg)
    #     # Wait for last message to arrive
    #     # await reader.get_message()
    #     print('Done!')
    #
    #     # Clean-up
    #     notifier.stop()
    #     can0.shutdown()
    #
    # def print_message(self, msg: can.Message) -> None:
    #     """Regular callback function. Can also be a coroutine."""
    #     print(msg)

    def goToSettings(self, fileName=None):
        if hasattr(self, 'setupWidget'):
            self.stack.removeWidget(self.setupWidget)
        self.setupWidget = SetupWidget(self.node, fileName=fileName,plugin=self,context=self.context)
        self.stack.addWidget(self.setupWidget)
        self.stack.setCurrentIndex(1)
