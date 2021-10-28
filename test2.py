import sys
from PySide6 import QtGui, QtCore, QtWidgets
from PySide6.QtWidgets import QBoxLayout, QGridLayout, QPushButton, QSizePolicy, QStyleOptionButton, QStyle, \
    QApplication


class myContainter(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super(myContainter, self).__init__(parent)

        icon = QtGui.QIcon('settings.png')

        grid = QGridLayout()

        for i in range(3):
            button = myPushButton()
            button.setIcon(icon)

            grid.addWidget(button, i, 0)

            grid.setRowStretch(i, i)

        self.setLayout(grid)


class myPushButton(QPushButton):
    def __init__(self, label=None, parent=None):
        super(myPushButton, self).__init__(label, parent)

        self.pad = 4  # padding between the icon and the button frame
        self.minSize = 8  # minimum size of the icon

        sizePolicy = QSizePolicy(QSizePolicy.Expanding,
                                 QSizePolicy.Expanding)
        self.setSizePolicy(sizePolicy)

    def paintEvent(self, event):
        qp = QtGui.QPainter()
        qp.begin(self)

        # ---- get default style ----

        opt = QStyleOptionButton()
        self.initStyleOption(opt)

        # ---- scale icon to button size ----

        Rect = opt.rect

        h = Rect.height()
        w = Rect.width()
        iconSize = max(min(h, w) - 2 * self.pad, self.minSize)

        opt.iconSize = QtCore.QSize(iconSize, iconSize)

        # ---- draw button ----

        self.style().drawControl(QStyle.CE_PushButton, opt, qp, self)

        qp.end()


if __name__ == '__main__':
    app = QApplication(sys.argv)

    instance = myContainter()
    instance.show()

    sys.exit(app.exec_())
