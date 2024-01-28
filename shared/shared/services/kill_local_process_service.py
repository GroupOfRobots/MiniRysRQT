import re
import subprocess
import threading

from python_qt_binding.QtCore import QThread

from .local_privileges_service import LocalPrivilegesService

INCORRECT_PASSWORD_PATTERN = r'sudo: (\d+) incorrect password attempt'


class KillLocalProcessService:
    @staticmethod
    def checkIfProcessWasKilledWhenUserHadNotPrivileges(process):
        output = str(process)
        isWrongPassword = re.search(INCORRECT_PASSWORD_PATTERN, output)

        if isWrongPassword:
            LocalPrivilegesService.wrongPasswordAlert()
            return False
        return True

    @staticmethod
    def killProcessCommand(pid):
        process = subprocess.Popen(['sudo', '-S', 'kill', '-9', str(pid)], stdin=subprocess.PIPE,
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        process.communicate()
        process.wait()

    @staticmethod
    def kill(pid):
        try:
            if LocalPrivilegesService.hasPrivileges():
                commandThread = threading.Thread(target=lambda: KillLocalProcessService.killProcessCommand(pid))
                commandThread.start()
            else:
                LocalPrivilegesService.addPrivilages("inputDialogText")
                commandThread = threading.Thread(target=lambda :KillLocalProcessService.killProcessCommand(pid))
                commandThread.start()
        except Exception as exception:
            print("Exception killProcess")
            print(exception)


class KillLocalProcessThread(QThread):
    pid=None
    def __init__(self, pid):
        super(QThread, self).__init__()
        self.pid=pid

    def run(self):
        KillLocalProcessService.killProcessCommand(self.pid)