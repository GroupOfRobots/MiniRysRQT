# from PyQt6 import QtWidgets
# from PyQt6.QtWidgets import QApplication, QMainWindow
# import sys
#
# from controlPanel2 import Ui_MainWindow
#
#
# def window():
#     app = QApplication(sys.argv)
#     win = QMainWindow()
#     win.setGeometry(330,222,300,300)
#     win.setWindowTitle("start")
#
#     pushButton = QtWidgets.QPushButton(win)
#     pushButton.setGeometry(72, 127, 111, 221)
#     pushButton.sizeHint()
#     pushButton.setObjectName("pushButton")
#
#     win.show()
#     sys.exit(app.exec())
#
# # window()
# # print("test")
from PySide6 import QtWidgets

from controlPanel2 import Ui_MainWindow

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
