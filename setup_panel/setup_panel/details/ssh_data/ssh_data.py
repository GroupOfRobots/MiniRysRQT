
from python_qt_binding.QtWidgets import QWidget

class SshData(QWidget):
    def __init__(self, widget):
        super(SshData, self).__init__()
        self.widget = widget
        self.sshData =self.widget.data['ssh']
        self.setSshData()

    def setSshData(self):
        self.widget.sshHostInputUI.setText(self.sshData['host'])
        self.widget.sshPortInputUI.setText(self.sshData['port'])
        self.widget.sshUsernameInputUI.setText(self.sshData['username'])
        self.widget.sshPasswordInputUI.setText(self.sshData['password'])

    def hostEdited(self):
        self.widget.data['ssh']['host'] = self.widget.sshHostInputUI.text()

    def portEdited(self):
        self.widget.data['ssh']['port'] = self.widget.sshPortInputUI.text()

    def usernameEdited(self):
        self.widget.data['ssh']['username'] = self.widget.sshUsernameInputUI.text()

    def passwordEdited(self):
        self.widget.data['ssh']['password'] = self.widget.sshPasswordInputUI.text()
