import subprocess

from python_qt_binding.QtCore import pyqtSignal, QObject
from python_qt_binding.QtWidgets import QInputDialog, QLineEdit, QMessageBox

CHECK_PRIVILEGES_COMMAND = "sudo -n true 2>/dev/null && echo Privileges active || echo Privileges inactive"


class LocalPrivilegesService(QObject):
    getPasswordInputDialog = pyqtSignal(bool, name="getPasswordInputDialog")

    @staticmethod
    def showInputDialog(inputDialogText='Password:'):
        password, ok = QInputDialog.getText(None, 'PASSWORD', inputDialogText, QLineEdit.Password)
        print(ok, "ok")
        if ok:
            return password
        return False

    @staticmethod
    def wrongPasswordAlert():
        msg = QMessageBox()
        msg.setWindowTitle("WRONG PASSWORD")
        msg.setText("Password was incorrect")
        msg.setIcon(QMessageBox.Warning)
        msg.show()

        msg.exec_()

    @staticmethod
    def onPasswordCancel():
        msg = QMessageBox()
        msg.setWindowTitle("PASSWORD CANCELED")
        msg.setText("You cancelled entering password \n"
                    "Hence you are now \n"
                    "On your own\n"
                    "Good luck :D")
        msg.setIcon(QMessageBox.Warning)
        msg.show()

        msg.exec_()

    @staticmethod
    def hasPrivileges():
        with subprocess.Popen(CHECK_PRIVILEGES_COMMAND, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              text=True) as process:
            output = [line.strip() for line in process.stdout]
            if output == ['Privileges active']:
                return True
            return False

    @staticmethod
    def getPassword(inputDialogText=None):
        if inputDialogText is None:
            password = LocalPrivilegesService.showInputDialog()
        else:
            password = LocalPrivilegesService.showInputDialog(inputDialogText)
        if password is False:
            return False
        passwordEnter = password + '\n'
        return bytes(passwordEnter, encoding='utf-8')

    @staticmethod
    def addPrivilages(inputDialogText=None):
        passwordEncoded = LocalPrivilegesService.getPassword(inputDialogText)

        if passwordEncoded is False:
            LocalPrivilegesService.onPasswordCancel()
            return

        # pwd is random command, because without it privileges are not added
        out, err = subprocess.Popen(['sudo', '-S', 'pwd'], stdin=subprocess.PIPE,
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate(input=passwordEncoded)
        outputString = out.decode("utf-8")
        if len(outputString) == 0:
            LocalPrivilegesService.addPrivilages("Wrong password\n"
                                                 "Try again")
