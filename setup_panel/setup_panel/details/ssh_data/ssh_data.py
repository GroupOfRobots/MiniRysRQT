from python_qt_binding.QtWidgets import QWidget


class SshData(QWidget):
    def __init__(self, widget):
        super(SshData, self).__init__()
        self.widget = widget
        self.setData(self.widget.data)

    def setData(self,data):
        self.sshData = data.get('ssh', {})
        self.widget.sshHostInputUI.setText(self.sshData.get('host'))
        self.widget.sshPortInputUI.setText(self.sshData.get('port'))
        self.widget.sshUsernameInputUI.setText(self.sshData.get('username'))
        self.widget.sshPasswordInputUI.setText(self.sshData.get('password'))

    def saveSshData(self):
        self.widget.data['ssh']['host'] = self.widget.sshHostInputUI.text()
        self.widget.data['ssh']['port'] = self.widget.sshPortInputUI.text()
        self.widget.data['ssh']['username'] = self.widget.sshUsernameInputUI.text()
        self.widget.data['ssh']['password'] = self.widget.sshPasswordInputUI.text()
