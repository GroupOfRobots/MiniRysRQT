from python_qt_binding.QtWidgets import QWidget

class SshData(QWidget):
    def __init__(self, widget):
        super(SshData, self).__init__()
        self.widget = widget
        self.sshData = self.widget.data.get('ssh', {})
        self.setSshData()

        # self.connectElements()

    def setSshData(self):
        self.widget.sshHostInputUI.setText(self.sshData.get('host'))
        self.widget.sshPortInputUI.setText(self.sshData.get('port'))
        self.widget.sshUsernameInputUI.setText(self.sshData.get('username'))
        self.widget.sshPasswordInputUI.setText(self.sshData.get('password'))

    # def connectElements(self):
    #     self.widget.sshHostInputUI.editingFinished.connect(self.hostEdited)
    #     self.widget.sshPortInputUI.editingFinished.connect(self.portEdited)
    #     self.widget.sshUsernameInputUI.editingFinished.connect(self.usernameEdited)
    #     self.widget.sshPasswordInputUI.editingFinished.connect(self.passwordEdited)

    # def hostEdited(self):
    #     self.widget.data['ssh']['host'] = self.widget.sshHostInputUI.text()
    #
    # def portEdited(self):
    #     self.widget.data['ssh']['port'] = self.widget.sshPortInputUI.text()
    #
    # def usernameEdited(self):
    #     self.widget.data['ssh']['username'] = self.widget.sshUsernameInputUI.text()
    #
    # def passwordEdited(self):
    #     self.widget.data['ssh']['password'] = self.widget.sshPasswordInputUI.text()

    def saveSshData(self):
        self.widget.data['ssh']['host'] = self.widget.sshHostInputUI.text()
        self.widget.data['ssh']['port'] = self.widget.sshPortInputUI.text()
        self.widget.data['ssh']['username'] = self.widget.sshUsernameInputUI.text()
        self.widget.data['ssh']['password'] = self.widget.sshPasswordInputUI.text()
