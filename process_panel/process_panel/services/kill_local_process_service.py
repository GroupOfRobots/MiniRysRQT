import re
import subprocess

from python_qt_binding.QtWidgets import QInputDialog, QLineEdit, QMessageBox

CHECK_PRIVILEGES_COMMAND = "sudo -n true 2>/dev/null && echo Privileges active || echo Privileges inactive"
INCORRECT_PASSWORD_PATTERN = r'sudo: (\d+) incorrect password attempt'


class KillLocalProcessService:
    @staticmethod
    def showInputDialog():
        password, ok = QInputDialog.getText(None, 'PASSWORD', 'Password:', QLineEdit.Password)
        if ok:
            return password

    @staticmethod
    def wrongPasswordAlert():
        msg = QMessageBox()
        msg.setWindowTitle("WRONG PASSWORD")
        msg.setText("Password was incorrect")
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
    def getPassword():
        password = KillLocalProcessService.showInputDialog()
        passwordEnter = password + '\n'
        return bytes(passwordEnter, encoding='utf-8')

    @staticmethod
    def checkIfProcessWasKilledWhenUserHadNotPrivileges(process):
        output = str(process)
        isWrongPassword = re.search(INCORRECT_PASSWORD_PATTERN, output)

        if isWrongPassword:
            KillLocalProcessService.wrongPasswordAlert()
            return False
        else:
            return True

    @staticmethod
    def killProcessCommand(pid):
        return subprocess.Popen(['sudo', '-S', 'kill', '-9', str(pid)], stdin=subprocess.PIPE,
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    @staticmethod
    def kill(pid):
        try:
            if KillLocalProcessService.hasPrivileges():
                KillLocalProcessService.killProcessCommand(pid)
                return True
            else:
                passwordEncoded = KillLocalProcessService.getPassword()
                process = KillLocalProcessService.killProcessCommand(pid).communicate(input=passwordEncoded)
                return KillLocalProcessService.checkIfProcessWasKilledWhenUserHadNotPrivileges(process)
        except Exception as exception:
            print("Exception killProcess")
            print(exception)
