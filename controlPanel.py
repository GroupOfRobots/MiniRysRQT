# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'form.ui'
##
## Created by: Qt User Interface Compiler version 6.2.0
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################
import sys

from PySide6 import QtWidgets
from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QCheckBox, QFrame, QGroupBox,
    QHBoxLayout, QLabel, QPushButton, QSizePolicy,
    QToolButton, QVBoxLayout, QWidget)

class Ui_Control(object):
    def setupUi(self, Control):
        if not Control.objectName():
            Control.setObjectName(u"Control")
        Control.setWindowModality(Qt.NonModal)
        Control.resize(719, 656)
        sizePolicy = QSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(Control.sizePolicy().hasHeightForWidth())
        Control.setSizePolicy(sizePolicy)
        Control.setAutoFillBackground(False)
        Control.setStyleSheet(u"background-color: rgb(255, 255, 255)")
        self.verticalLayout_2 = QVBoxLayout(Control)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.toolbarLayout = QHBoxLayout()
        self.toolbarLayout.setObjectName(u"toolbarLayout")
        self.toolbarBox = QGroupBox(Control)
        self.toolbarBox.setObjectName(u"toolbarBox")
        sizePolicy1 = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.toolbarBox.sizePolicy().hasHeightForWidth())
        self.toolbarBox.setSizePolicy(sizePolicy1)
        self.toolbarBox.setMinimumSize(QSize(0, 0))
        self.toolbarBox.setMaximumSize(QSize(16777215, 50))
        self.toolbarBox.setStyleSheet(u"QGroupBox {\n"
"    background-color: rgb(220, 200, 200);\n"
"}")
        self.horizontalLayout_7 = QHBoxLayout(self.toolbarBox)
        self.horizontalLayout_7.setObjectName(u"horizontalLayout_7")
        self.robotLabel = QLabel(self.toolbarBox)
        self.robotLabel.setObjectName(u"robotLabel")
        self.robotLabel.setStyleSheet(u"background-color: rgb(220, 200, 200);")

        self.horizontalLayout_7.addWidget(self.robotLabel)

        self.keyboard = QCheckBox(self.toolbarBox)
        self.keyboard.setObjectName(u"keyboard")
        self.keyboard.setBaseSize(QSize(0, 0))
        self.keyboard.setStyleSheet(u"background-color: rgb(220, 200, 200);")

        self.horizontalLayout_7.addWidget(self.keyboard, 0, Qt.AlignRight)

        self.toolButton = QToolButton(self.toolbarBox)
        self.toolButton.setObjectName(u"toolButton")
        self.toolButton.setCursor(QCursor(Qt.PointingHandCursor))
        self.toolButton.setStyleSheet(u"background-color:rgb(220, 220, 220);\n"
"border: none;\n"
"")
        icon = QIcon()
        icon.addFile(u"imgs/settings1.png", QSize(), QIcon.Normal, QIcon.Off)
        self.toolButton.setIcon(icon)
        self.toolButton.setCheckable(False)

        self.horizontalLayout_7.addWidget(self.toolButton)


        self.toolbarLayout.addWidget(self.toolbarBox)


        self.verticalLayout_2.addLayout(self.toolbarLayout)

        self.line = QFrame(Control)
        self.line.setObjectName(u"line")
        self.line.setSizeIncrement(QSize(0, 1))
        self.line.setFrameShape(QFrame.HLine)
        self.line.setFrameShadow(QFrame.Sunken)

        self.verticalLayout_2.addWidget(self.line)

        self.buttonsLayout = QVBoxLayout()
        self.buttonsLayout.setSpacing(12)
        self.buttonsLayout.setObjectName(u"buttonsLayout")
        self.buttonsFirstRow = QHBoxLayout()
        self.buttonsFirstRow.setObjectName(u"buttonsFirstRow")
        self.emptyFrameLeft = QFrame(Control)
        self.emptyFrameLeft.setObjectName(u"emptyFrameLeft")
        self.emptyFrameLeft.setFrameShape(QFrame.StyledPanel)
        self.emptyFrameLeft.setFrameShadow(QFrame.Raised)

        self.buttonsFirstRow.addWidget(self.emptyFrameLeft)

        self.forward_button = QPushButton(Control)
        self.forward_button.setObjectName(u"forward_button")
        sizePolicy2 = QSizePolicy(QSizePolicy.Minimum, QSizePolicy.Minimum)
        sizePolicy2.setHorizontalStretch(0)
        sizePolicy2.setVerticalStretch(0)
        sizePolicy2.setHeightForWidth(self.forward_button.sizePolicy().hasHeightForWidth())
        self.forward_button.setSizePolicy(sizePolicy2)
        self.forward_button.setSizeIncrement(QSize(0, 0))
        self.forward_button.setCursor(QCursor(Qt.PointingHandCursor))
        self.forward_button.setStyleSheet(u"background-color: rgb(200, 200, 200);\n"
"border: none;")
        icon1 = QIcon()
        icon1.addFile(u"imgs/forwardArrow.png", QSize(), QIcon.Normal, QIcon.Off)
        self.forward_button.setIcon(icon1)
        self.forward_button.setIconSize(QSize(150, 150))

        self.buttonsFirstRow.addWidget(self.forward_button)

        self.emptyFrameRight = QFrame(Control)
        self.emptyFrameRight.setObjectName(u"emptyFrameRight")
        self.emptyFrameRight.setFrameShape(QFrame.StyledPanel)
        self.emptyFrameRight.setFrameShadow(QFrame.Raised)

        self.buttonsFirstRow.addWidget(self.emptyFrameRight)


        self.buttonsLayout.addLayout(self.buttonsFirstRow)

        self.buttonsSecondRow = QHBoxLayout()
        self.buttonsSecondRow.setObjectName(u"buttonsSecondRow")
        self.leftButton = QPushButton(Control)
        self.leftButton.setObjectName(u"leftButton")
        sizePolicy1.setHeightForWidth(self.leftButton.sizePolicy().hasHeightForWidth())
        self.leftButton.setSizePolicy(sizePolicy1)
        self.leftButton.setSizeIncrement(QSize(1, 1))
        self.leftButton.setBaseSize(QSize(10, 10))
        self.leftButton.setCursor(QCursor(Qt.PointingHandCursor))
        self.leftButton.setAutoFillBackground(False)
        self.leftButton.setStyleSheet(u"background-color: rgb(200, 200, 200);\n"
"border: none;")
        self.leftButton.setIcon(icon1)
        self.leftButton.setIconSize(QSize(150, 150))

        self.buttonsSecondRow.addWidget(self.leftButton)

        self.backwardButton = QPushButton(Control)
        self.backwardButton.setObjectName(u"backwardButton")
        sizePolicy1.setHeightForWidth(self.backwardButton.sizePolicy().hasHeightForWidth())
        self.backwardButton.setSizePolicy(sizePolicy1)
        self.backwardButton.setCursor(QCursor(Qt.PointingHandCursor))
        self.backwardButton.setStyleSheet(u"background-color: rgb(200, 200, 200);\n"
"border: none;")
        self.backwardButton.setIcon(icon1)
        self.backwardButton.setIconSize(QSize(150, 150))

        self.buttonsSecondRow.addWidget(self.backwardButton)

        self.rightButton = QPushButton(Control)
        self.rightButton.setObjectName(u"rightButton")
        sizePolicy1.setHeightForWidth(self.rightButton.sizePolicy().hasHeightForWidth())
        self.rightButton.setSizePolicy(sizePolicy1)
        self.rightButton.setCursor(QCursor(Qt.PointingHandCursor))
        self.rightButton.setStyleSheet(u"background-color: rgb(200, 200, 200);\n"
"border: none;")
        self.rightButton.setIcon(icon1)
        self.rightButton.setIconSize(QSize(150, 150))

        self.buttonsSecondRow.addWidget(self.rightButton)


        self.buttonsLayout.addLayout(self.buttonsSecondRow)


        self.verticalLayout_2.addLayout(self.buttonsLayout)


        self.retranslateUi(Control)

        QMetaObject.connectSlotsByName(Control)
    # setupUi

    def retranslateUi(self, Control):
        Control.setWindowTitle(QCoreApplication.translate("Control", u"Form", None))
        self.toolbarBox.setTitle("")
        self.robotLabel.setText(QCoreApplication.translate("Control", u"TextLabel", None))
        self.keyboard.setText(QCoreApplication.translate("Control", u"keyboard", None))
        self.toolButton.setText("")
        self.forward_button.setText("")
        self.leftButton.setText("")
        self.backwardButton.setText("")
        self.rightButton.setText("")
    # retranslateUi

class MainWindow(QtWidgets.QMainWindow, Ui_Control):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.setupUi(self)


app = QtWidgets.QApplication(sys.argv)

window = MainWindow()
window.show()
app.exec_()