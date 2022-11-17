# This Python file uses the following encoding: utf-8
import math
import os

# from PyQt5.QtWidgets import QGraphicsPixmapItem
# from PyQt5.QtGui import QImage
# from PyQt5.QtWidgets import QGraphicsScene
from python_qt_binding.QtCore import Qt, QPoint, QLineF
from python_qt_binding.QtGui import QPainter, QPen
from ament_index_python import get_resource
from python_qt_binding import loadUi
from shared.base_widget.base_widget import BaseWidget

from rclpy.node import Node
from minirys_msgs.msg import BatteryStatus, AngularPose
from sensor_msgs.msg import Range

from std_msgs.msg import Float32, String

import time
from collections import namedtuple

from python_qt_binding.QtWidgets import QGraphicsScene, QGraphicsPixmapItem, QFileDialog
from python_qt_binding.QtGui import QImage, QPixmap, QTransform


# from python_qt_binding.Qt import AspectRatioMode


class DashboardDistanceSensorsWidget(BaseWidget):
    def __init__(self, stack=None, node=Node):
        super(DashboardDistanceSensorsWidget, self).__init__()

        self.node = node
        self.predefineSubscribers()

        self.setRobotOnScreen()

        self.scene = QGraphicsScene()

        _, packagePath = get_resource('packages', 'dashboard_distance_sensors')
        file_name = os.path.join(packagePath, 'share', 'dashboard_distance_sensors', 'resource', 'imgs', 'rys.png')

        self.image_qt = QImage(file_name)
        self.image_qt = self.image_qt.scaled(500, 500, Qt.IgnoreAspectRatio)
        self.pic = QGraphicsPixmapItem()
        self.pic.setPixmap(QPixmap.fromImage(self.image_qt))
        self.pic.setOpacity(0.5)
        self.scene.addItem(self.pic)

        # text = self.scene.addText('')
        # text.setPos(100, 200)
        # text.setHtml('<div>FRONT </div><div>b</div><div>c</div>')
        self.scene.addLine(0, 0, 100, 100)

        for index, subscriberParam in enumerate(self.subscriberParams):
            print(subscriberParam.textPlaceholder)
            line = self.scene.addLine(subscriberParam.QLineF)
            text = self.scene.addText(subscriberParam.textPlaceholder)
            subscriberParam = subscriberParam._replace(line=line)
            subscriberParam = subscriberParam._replace(text=text)

            subscriberParam.text.setPos(subscriberParam.QPoint)
            self.subscriberParams[index] = subscriberParam

        self.graphicsView.setScene(self.scene)
        self.graphicsView.show()

    def loadUI(self):
        _, packagePath = get_resource('packages', 'dashboard_distance_sensors')
        uiFile = os.path.join(packagePath, 'share', 'dashboard_distance_sensors', 'resource',
                              'dashboard_distance_sensors.ui')
        loadUi(uiFile, self)

    def predefineSubscribers(self):
        self.topSensorSubsciberParam = SubscriberParam(None, Range, '/internal/distance_0',
                                                       self.topDistanceSensorCallback, QPoint(250, 0), None, 'TOP',
                                                       QLineF(10, 20, 30, 40), None)

        self.bottomSensorSubsciberParam = SubscriberParam(None, Range, '/internal/distance_1',
                                                          self.bottomDistanceSensorCallback, QPoint(250, 400),
                                                          None, 'BOTTOM', QLineF(10, 20, 30, 40), None)

        self.rightSensorSubsciberParam = SubscriberParam(None, Range, '/internal/distance_2',
                                                         self.rightDistanceSensorCallback, QPoint(400, 250),
                                                         None, 'RIGHT', QLineF(10, 20, 30, 40), None)

        self.backSensorSubsciberParam = SubscriberParam(None, Range, '/internal/distance_3',
                                                        self.backDistanceSensorCallback,
                                                        QPoint(400, 200),
                                                        None, 'BACK', QLineF(10, 20, 30, 40), None)

        self.frontSensorSubsciberParam = SubscriberParam(None, Range, '/internal/distance_4',
                                                         self.frontDistanceSensorCallback,
                                                         QPoint(200, 200),
                                                         None, 'FRONT', QLineF(10, 20, 30, 40), None)

        self.leftSensorSubsciberParam = SubscriberParam(None, Range, '/internal/distance_5',
                                                        self.leftDistanceSensorCallback,
                                                        QPoint(0, 250), None, 'LEFT',
                                                        QLineF(10, 20, 30, 40), None)

        self.subscriberParams = [
            self.topSensorSubsciberParam,
            self.bottomSensorSubsciberParam,
            self.rightSensorSubsciberParam,
            self.backSensorSubsciberParam,
            self.frontSensorSubsciberParam,
            self.leftSensorSubsciberParam]

    def resetSubscribers(self):
        for subscriberParam in self.subscriberParams:

            # print(subscriberParam)
            # print(subscriberParam.subscriber)
            # print("")
            subscriber = subscriberParam.subscriber
            if subscriber is not None:
                self.node.destroy_subscription(subscriber)

    def initializeSubscribers(self):
        self.resetSubscribers()
        for index, subscriberParam in enumerate(self.subscriberParams):
            subscriberInstance = self.node.create_subscription(subscriberParam.messageType,
                                                               self.namespace + subscriberParam.topic,
                                                               subscriberParam.callback, 10)
            subscriberParam = subscriberParam._replace(subscriber=subscriberInstance)
            self.subscriberParams[index] = subscriberParam

    def initializeRobotSettings(self):
        self.initializeSubscribers()

    def topDistanceSensorCallback(self, event):
        print(event)
        # self.topDistanceLcdUI.display(event.range)

    def bottomDistanceSensorCallback(self, event):
        # self.bottomDistanceLcdUI.display(event.range)

        print(event)

    def rightDistanceSensorCallback(self, event):
        # self.rightDistanceLcdUI.display(event.range)
        print(event)

    def backDistanceSensorCallback(self, event):
        # self.backDistanceLcdUI.display(event.range)
        print(event)

    def frontDistanceSensorCallback(self, event):
        # self.frontDistanceLcdUI.display(event.range)
        print(event)

    def leftDistanceSensorCallback(self, event):
        # self.leftDistanceLcdUI.display(event.range)
        print(event)

    def backSensorCallback(self, event):
        # self.backDistanceLcdUI.display(event.range)
        print(event)

    def resizeEvent(self, event):

        graphicsViewSize = self.graphicsView.size()

        self.scene.setSceneRect(0, 0, graphicsViewSize.width(), graphicsViewSize.height())
        self.image_qt = self.image_qt.scaled(graphicsViewSize.width(), graphicsViewSize.height(),
                                             Qt.IgnoreAspectRatio)

        transform = QTransform()
        xScaleFactor = graphicsViewSize.width() / 500
        yScaleFactor = graphicsViewSize.height() / 500
        transform.scale(xScaleFactor, yScaleFactor)
        self.pic.setTransform(transform)

        for subscriberParam in self.subscriberParams:
            line = subscriberParam.QLineF
            textPoint = subscriberParam.QPoint
            newX = int(textPoint.x() * xScaleFactor)
            newY = int(textPoint.y() * yScaleFactor)

            textNewPoint = QPoint(newX, newY)

            subscriberParam.text.setPos(textNewPoint)


SubscriberParam = namedtuple('SubscriberParam',
                             ["subscriber", "messageType", "topic", "callback", 'QPoint', 'text', 'textPlaceholder',
                              'QLineF', 'line'])
